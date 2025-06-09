#include <rclcpp/rclcpp.hpp>
#include <pzb_msgs/msg/lift_state.hpp>
#include <pzb_msgs/srv/set_lift.hpp>

#include <thread>
#include <mutex>
#include <chrono>
#include <map>
#include <atomic>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

class FPGANode : public rclcpp::Node
{
public:
    FPGANode() : Node("fpga_node"), serial_fd_(-1), current_state_(pzb_msgs::msg::LiftState::IDLE), 
                 should_stop_(false)
    {
        RCLCPP_INFO(this->get_logger(), "FPGA NODE Initialized!");
        
        // Declarar parámetros
        this->declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
        this->declare_parameter<int>("baudrate", 9600);
        this->declare_parameter<double>("timeout", 0.1);
        
        // Obtener parámetros
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        
        // Inicializar mapeos
        initializeMappings();
        
        // Configurar puerto serie
        if (setupSerial(serial_port, baudrate)) {
            RCLCPP_INFO(this->get_logger(), "Puerto serie abierto: %s @ %d baud", 
                       serial_port.c_str(), baudrate);
        } else {
            RCLCPP_WARN(this->get_logger(), "Puerto serie no disponible: %s", serial_port.c_str());
        }
        
        // Publisher para el estado actual del lift
        state_publisher_ = this->create_publisher<pzb_msgs::msg::LiftState>(
            "current_lift_state", 10);
        
        // Servicio para establecer el estado deseado
        set_lift_service_ = this->create_service<pzb_msgs::srv::SetLift>(
            "set_lift_state",
            std::bind(&FPGANode::setLiftCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Timer para publicar el estado actual periódicamente
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FPGANode::publishCurrentState, this));
        
        // Iniciar hilo para lectura continua del puerto serie SOLO si está disponible
        if (serial_fd_ != -1) {
            serial_reading_thread_ = std::thread(&FPGANode::serialReadingLoop, this);
            RCLCPP_INFO(this->get_logger(), "Lectura serie iniciado");
        }
        
        // Publicar estado inicial
        publishCurrentState();
        
        RCLCPP_INFO(this->get_logger(), "FPGA Node Initialized OK");
        RCLCPP_INFO(this->get_logger(), "Servicios disponibles:");
        RCLCPP_INFO(this->get_logger(), "  - ros2 service call /set_lift_state pzb_msgs/srv/SetLift \"{desired_state: 0}\"");
        RCLCPP_INFO(this->get_logger(), "Tópicos publicados:");
        RCLCPP_INFO(this->get_logger(), "  - /current_lift_state (pzb_msgs/msg/LiftState)");
        RCLCPP_INFO(this->get_logger(), "Iniciando lectura continua del puerto serie...");
    }
    
    ~FPGANode()
    {
        RCLCPP_INFO(this->get_logger(), "Closing FPGA Node...");
        
        // Señalar al hilo que debe terminar
        should_stop_ = true;
        
        if (serial_reading_thread_.joinable()) {
            serial_reading_thread_.join();
        }
        
        // Cerrar puerto serie
        if (serial_fd_ != -1) {
            close(serial_fd_);
            RCLCPP_INFO(this->get_logger(), "Puerto serie cerrado");
        }
    }

private:
    bool setupSerial(const std::string& port, int baudrate)
    {
        struct termios tty;
        
        // Abrir puerto serie
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error abriendo puerto %s: %s", 
                        port.c_str(), strerror(errno));
            return false;
        }
        
        // Configurar puerto serie
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error tcgetattr: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        
        // Configurar baudrate
        speed_t speed;
        switch (baudrate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Baudrate no soportado: %d", baudrate);
                close(serial_fd_);
                serial_fd_ = -1;
                return false;
        }
        
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        
        // Configurar 8N1
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        
        // Configurar modo raw
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        
        // Timeout settings
        tty.c_cc[VTIME] = 1;
        tty.c_cc[VMIN] = 0;
        
        // Aplicar configuración
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error tcsetattr: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        
        return true;
    }
    
    void initializeMappings()
    {
        fpga_to_ros_state_ = {
            {'I', pzb_msgs::msg::LiftState::IDLE},
            {'P', pzb_msgs::msg::LiftState::PRELOAD},
            {'L', pzb_msgs::msg::LiftState::LOAD},
            {'T', pzb_msgs::msg::LiftState::RAISE},
            {'E', pzb_msgs::msg::LiftState::UNLOAD},
            {'U', pzb_msgs::msg::LiftState::IDLE},
            {'D', pzb_msgs::msg::LiftState::IDLE}
        };
        
        ros_to_fpga_command_ = {
            {pzb_msgs::msg::LiftState::IDLE, 'i'},
            {pzb_msgs::msg::LiftState::PRELOAD, 'p'},
            {pzb_msgs::msg::LiftState::LOAD, 'l'},
            {pzb_msgs::msg::LiftState::RAISE, 't'},
            {pzb_msgs::msg::LiftState::UNLOAD, 'e'}
        };
        
        state_names_ = {
            {pzb_msgs::msg::LiftState::IDLE, "IDLE"},
            {pzb_msgs::msg::LiftState::PRELOAD, "PRELOAD"},
            {pzb_msgs::msg::LiftState::LOAD, "LOAD"},
            {pzb_msgs::msg::LiftState::RAISE, "RAISE"},
            {pzb_msgs::msg::LiftState::UNLOAD, "UNLOAD"}
        };
    }
    
    // 🔧 HILO DE LECTURA SERIAL - SIN MUTEX para evitar deadlock
    void serialReadingLoop()
    {
        RCLCPP_INFO(this->get_logger(), "Hilo de lectura serie activo");
        
        while (!should_stop_ && rclcpp::ok() && serial_fd_ != -1) {
            char buffer[256];
            ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer));
            
            if (bytes_read > 0) {
                for (ssize_t i = 0; i < bytes_read; i++) {
                    char received_char = buffer[i];
                    
                    auto it = fpga_to_ros_state_.find(received_char);
                    if (it != fpga_to_ros_state_.end()) {
                        int8_t new_state = it->second;
                        
                        // ATOMIC en lugar de MUTEX para evitar deadlock
                        int8_t old_state = current_state_.exchange(new_state);
                        
                        if (new_state != old_state) {
                            RCLCPP_INFO(this->get_logger(), 
                                       "Estado actualizado desde FPGA: %c -> %s", 
                                       received_char, getStateName(new_state).c_str());
                        }
                    } else if (received_char != '\0' && received_char != '\n' && received_char != '\r') {
                        RCLCPP_DEBUG(this->get_logger(), 
                                    "Dato recibido no reconocido: %c (0x%02X)", 
                                    received_char, static_cast<unsigned char>(received_char));
                    }
                }
            } else if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(this->get_logger(), "Error leyendo puerto serie: %s", strerror(errno));
            }
            
            // SLEEP MÁS LARGO para reducir carga CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        RCLCPP_INFO(this->get_logger(), "Hilo de lectura serie terminado");
    }
    
    // PUBLISH SIN MUTEX - usa atomic
    void publishCurrentState()
    {
        auto msg = pzb_msgs::msg::LiftState();
        msg.state = current_state_.load();  // Atomic read
        state_publisher_->publish(msg);
    }
    
    // CALLBACK SIN BLOQUEO
    void setLiftCallback(const std::shared_ptr<pzb_msgs::srv::SetLift::Request> request,
                        std::shared_ptr<pzb_msgs::srv::SetLift::Response> response)
    {
        (void)response;
        
        RCLCPP_INFO(this->get_logger(), "CALLBACK - desired_state: %d", 
                   request->desired_state);
        
        int8_t desired_state = request->desired_state;
        
        // Validar estado
        auto it = ros_to_fpga_command_.find(desired_state);
        if (it == ros_to_fpga_command_.end()) {
            RCLCPP_ERROR(this->get_logger(), "Estado inválido: %d", desired_state);
            return;
        }
        
        char fpga_command = it->second;
        
        // ESCRITURA UART SIN MUTEX
        if (serial_fd_ != -1) {
            ssize_t bytes_written = write(serial_fd_, &fpga_command, 1);
            if (bytes_written == 1) {
                RCLCPP_INFO(this->get_logger(), 
                           "Comando enviado al FPGA: %c (Estado: %s)", 
                           fpga_command, getStateName(desired_state).c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Error escribiendo al puerto serie: %s", 
                            strerror(errno));
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Puerto serie no disponible - simulando comando: %c (Estado: %s)", 
                       fpga_command, getStateName(desired_state).c_str());
        }
        
        // ACTUALIZAR ESTADO CON ATOMIC
        current_state_.store(desired_state);
        RCLCPP_INFO(this->get_logger(), "Estado actualizado localmente: %s", 
                   getStateName(desired_state).c_str());
    }
    
    std::string getStateName(int8_t state)
    {
        auto it = state_names_.find(state);
        if (it != state_names_.end()) {
            return it->second;
        }
        return "UNKNOWN(" + std::to_string(state) + ")";
    }
    
    // MIEMBROS PRIVADOS - Sin mutex problemático
    int serial_fd_;
    std::atomic<int8_t> current_state_;              // ✅ ATOMIC en lugar de mutex
    std::atomic<bool> should_stop_;                  // ✅ ATOMIC para control de hilo
    
    rclcpp::Publisher<pzb_msgs::msg::LiftState>::SharedPtr state_publisher_;
    rclcpp::Service<pzb_msgs::srv::SetLift>::SharedPtr set_lift_service_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    std::thread serial_reading_thread_;
    
    // Mapeos (solo lectura, no necesitan protección)
    std::map<char, int8_t> fpga_to_ros_state_;
    std::map<int8_t, char> ros_to_fpga_command_;
    std::map<int8_t, std::string> state_names_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto fpga_node = std::make_shared<FPGANode>();
        rclcpp::spin(fpga_node);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    
    rclcpp::shutdown();
    return 0;
}
