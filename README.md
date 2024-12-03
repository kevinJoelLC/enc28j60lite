# TCP/IP Library for ENC28J60 Ported to XC8 🖧📚

Welcome to the **TCP/IP Library for ENC28J60**, designed to work seamlessly with Microchip's XC8 compiler. This library provides lightweight and efficient TCP/IP stack functionality for PIC microcontrollers with ENC28J60 Ethernet controllers. 🚀

---

## 📌 Features

- 🌐 **UDP Send Functionality**: Streamlined implementation for sending UDP packets.
- ⚡ **Optimized for XC8**: Built to work efficiently with the XC8 compiler.
- 📡 **ENC28J60 Driver Support**: Complete integration with the ENC28J60 Ethernet controller.
- 📜 **Modular Design**: Easy to integrate and extend for other TCP/IP functionalities.
- 🛠️ **Customizable**: Tailored for projects requiring lightweight networking capabilities.

---

## 🛑 Requirements

- 🔧 **Microcontroller**: Compatible with PIC microcontrollers (tested with PIC16F877A).
- 📘 **Compiler**: XC8 Compiler.
- ⏲️ **Clock Frequency**: Requires a clock of at least 20 MHz for proper operation.
- 📦 **Hardware**: ENC28J60 Ethernet controller.

---

## 📂 Project Structure

```plaintext
📦 Project Folder
├── 📜 README.md
├── 📁 src
│   ├── main.c
│   ├── tcpip_library.c
│   └── tcpip_library.h
├── 📁 docs
│   ├── user_manual.pdf
│   └── api_reference.md
└── 📁 examples
    ├── udp_send_example.c
    └── debug_example.c
```

---

## 🚀 Getting Started

### 1️⃣ Setup
1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/tcpip-library-enc28j60-xc8.git
   ```
2. Open the project in your IDE (e.g., MPLAB X IDE).
3. Configure the project to use the XC8 compiler.

### 2️⃣ Usage
1. Include the library header in your main file:
   ```c
   #include "tcpip_library.h"
   ```
2. Initialize the ENC28J60 module:
   ```c
   ENC28J60_Init();
   ```
3. Send a UDP packet:
   ```c
   UDP_Send("192.168.1.100", 12345, "Hello, World!");
   ```

---

## 📖 Documentation

Comprehensive documentation is available in the `docs` folder:
- **User Manual**: Detailed setup and configuration instructions.
- **API Reference**: Overview of all available functions.

---

## 🧪 Examples

### Sending a UDP Packet 🌐
Check out the `examples/udp_send_example.c` file for a step-by-step implementation of sending a UDP packet using this library.

### Debugging 🛠️
Refer to the `examples/debug_example.c` file to see how to debug the library and monitor communication.

---

## 🛡️ License

This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for details.

---

## 🤝 Contributing

Contributions are welcome! If you have suggestions or improvements, feel free to fork this repository, create a branch, and submit a pull request.

1. Fork the repository.
2. Create a feature branch:
   ```bash
   git checkout -b feature-branch
   ```
3. Commit your changes:
   ```bash
   git commit -m "Add a new feature"
   ```
4. Push to your branch:
   ```bash
   git push origin feature-branch
   ```
5. Open a pull request.

---

## 🛠️ Development Tools

- **MPLAB X IDE** 🖥️
- **XC8 Compiler** 🛠️
- **ENC28J60 Module** 🌐

---

## 📬 Contact

For questions or support, please contact:
- **Author**: Kevin Joel López Carrillo
- **Email**: [your_email@example.com](mailto:your_email@example.com)
- **GitHub**: [yourusername](https://github.com/yourusername)

---

### Happy Networking! 🖧✨
