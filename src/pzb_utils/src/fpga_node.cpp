// UBICACIÓN: ~/pzb_ws/src/pzb_utils/src/fpga_node.cpp
// VERSIÓN SIN LIBRERÍA SERIAL - USA UART NATIVO DE LINUX

#include <rclcpp/rclcpp.hpp>
#include <pzb_msgs/msg/lift_state.hpp>
#include "pzb_msgs/srv/set_lift.hpp"
#include <thread>
#include <mutex>
#include <chrono>
#include <map>

// Headers para UART nativo de Linux
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

class FPGANode : public rclcpp::Node
{
public:
    FPGANode() : Node("fpga_node"), current_state_(pzb_msgs::msg::LiftState::IDLE), serial_fd_(-1)
    {
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
            RCLCPP_ERROR(this->get_logger(), "Error al abrir puerto serie: %s", serial_port.c_str());
            return;
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
        
        // Iniciar hilo para lectura continua del puerto serie
        serial_reading_thread_ = std::thread(&FPGANode::serialReadingLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "FPGA Node iniciado correctamente");
        RCLCPP_INFO(this->get_logger(), "Servicios disponibles:");
        RCLCPP_INFO(this->get_logger(), "  - ros2 service call /set_lift_state pzb_msgs/srv/SetLift \"{desired_state: 0}\"");
        RCLCPP_INFO(this->get_logger(), "Tópicos publicados:");
        RCLCPP_INFO(this->get_logger(), "  - /current_lift_state (pzb_msgs/msg/LiftState)");
    }
    
    ~FPGANode()
    {
        // Señalar al hilo que debe terminar
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
        // Abrir puerto serie
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error abriendo puerto %s: %s", 
                        port.c_str(), strerror(errno));
            return false;
        }
        
        // Configurar puerto serie
        struct termios tty;
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
        tty.c_cflag &= ~PARENB;        // Sin paridad
        tty.c_cflag &= ~CSTOPB;        // 1 stop bit
        tty.c_cflag &= ~CSIZE;         // Limpiar tamaño
        tty.c_cflag |= CS8;            // 8 bits
        tty.c_cflag &= ~CRTSCTS;       // Sin control de flujo
        tty.c_cflag |= CREAD | CLOCAL; // Habilitar lectura
        
        // Configurar modo raw
        tty.c_lflag &= ~ICANON;        // Modo raw
        tty.c_lflag &= ~ECHO;          // Sin echo
        tty.c_lflag &= ~ECHOE;         // Sin echo erase
        tty.c_lflag &= ~ECHONL;        // Sin echo newline
        tty.c_lflag &= ~ISIG;          // Sin interpretación de señales
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Sin control de flujo software
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        
        tty.c_oflag &= ~OPOST;         // Sin procesamiento de salida
        tty.c_oflag &= ~ONLCR;         // Sin conversión NL a CR-NL
        
        // Timeout settings
        tty.c_cc[VTIME] = 1;           // Timeout 0.1 segundos
        tty.c_cc[VMIN] = 0;            // Lectura no bloqueante
        
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
        // Mapeo de caracteres FPGA a estados ROS2
        fpga_to_ros_state_ = {
            {'I', pzb_msgs::msg::LiftState::IDLE},      // Idle
            {'P', pzb_msgs::msg::LiftState::PRELOAD},   // Preload
            {'E', pzb_msgs::msg::LiftState::LOAD},      // Delivery -> Load
            {'T', pzb_msgs::msg::LiftState::RAISE},     // Top -> Raise
            {'U', pzb_msgs::msg::LiftState::RAISE},     // Up/Increment -> Raise
            {'D', pzb_msgs::msg::LiftState::UNLOAD}     // Down/Decrement -> Unload
        };
        
        // Mapeo de estados ROS2 a comandos FPGA
        ros_to_fpga_command_ = {
            {pzb_msgs::msg::LiftState::IDLE, 'i'},
            {pzb_msgs::msg::LiftState::PRELOAD, 'p'},
            {pzb_msgs::msg::LiftState::LOAD, 'e'},      // Load -> delivery
            {pzb_msgs::msg::LiftState::RAISE, 't'},     // Raise -> top
            {pzb_msgs::msg::LiftState::UNLOAD, 'd'}     // Unload -> down/decrement
        };
        
        // Nombres de estados para logging
        state_names_ = {
            {pzb_msgs::msg::LiftState::IDLE, "IDLE"},
            {pzb_msgs::msg::LiftState::PRELOAD, "PRELOAD"},
            {pzb_msgs::msg::LiftState::LOAD, "LOAD"},
            {pzb_msgs::msg::LiftState::RAISE, "RAISE"},
            {pzb_msgs::msg::LiftState::UNLOAD, "UNLOAD"}
        };
    }
    
    void serialReadingLoop()
    {
        RCLCPP_INFO(this->get_logger(), "Iniciando lectura continua del puerto serie...");
        
        while (rclcpp::ok() && serial_fd_ != -1) {
            char buffer[256];
            ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer));
            
            if (bytes_read > 0) {
                for (ssize_t i = 0; i < bytes_read; i++) {
                    char received_char = buffer[i];
                    
                    auto it = fpga_to_ros_state_.find(received_char);
                    if (it != fpga_to_ros_state_.end()) {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        int8_t new_state = it->second;
                        
                        // Solo actualizar si el estado cambió
                        if (new_state != current_state_) {
                            current_state_ = new_state;
                            RCLCPP_INFO(this->get_logger(), 
                                       "Estado actualizado desde FPGA: %c -> %s", 
                                       received_char, getStateName(new_state).c_str());
                        }
                        
                        // Publicar inmediatamente cuando se recibe datos del FPGA
                        publishCurrentState();
                    } else if (received_char != '\0' && received_char != '\n' && received_char != '\r') {
                        RCLCPP_DEBUG(this->get_logger(), 
                                    "Dato recibido no reconocido: %c (0x%02X)", 
                                    received_char, static_cast<unsigned char>(received_char));
                    }
                }
            } else if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(this->get_logger(), "Error leyendo puerto serie: %s", strerror(errno));
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    void publishCurrentState()
    {
        auto msg = pzb_msgs::msg::LiftState();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            msg.state = current_state_;
        }
        state_publisher_->publish(msg);
    }
    
    void setLiftCallback(const std::shared_ptr<pzb_msgs::srv::SetLift::Request> request,
                        std::shared_ptr<pzb_msgs::srv::SetLift::Response> response)
    {
        int8_t desired_state = request->desired_state;
        
        // Validar que el estado deseado sea válido
        auto it = ros_to_fpga_command_.find(desired_state);
        if (it == ros_to_fpga_command_.end()) {
            RCLCPP_ERROR(this->get_logger(), "Estado inválido: %d", desired_state);
            return;
        }
        
        // Obtener el comando correspondiente para el FPGA
        char fpga_command = it->second;
        
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
            RCLCPP_ERROR(this->get_logger(), "Puerto serie no está abierto");
        }
    }
    
    std::string getStateName(int8_t state)
    {
        auto it = state_names_.find(state);
        if (it != state_names_.end()) {
            return it->second;
        }
        return "UNKNOWN(" + std::to_string(state) + ")";
    }
    
    // Miembros privados
    int serial_fd_;
    rclcpp::Publisher<pzb_msgs::msg::LiftState>::SharedPtr state_publisher_;
    rclcpp::Service<pzb_msgs::srv::SetLift>::SharedPtr set_lift_service_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    std::thread serial_reading_thread_;
    std::mutex state_mutex_;
    int8_t current_state_;
    
    // Mapeos
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