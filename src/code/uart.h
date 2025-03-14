#include <iostream>
#include <libserial/SerialPort.h>
#include <string.h>
#include "controlCenter.h"
using namespace LibSerial;

#define UsbFrameHead 0x42    // USB通信帧头
#define UsbFrameLengthMin 4  // USB通信帧最短长度（字节）
#define UsbFrameLengthMax 30 // USB通信帧最长长度（字节）

typedef union
{
    uint8_t U8_Buff[2];
    uint16_t U16;
} Bint16_Union;

typedef union
{
    uint8_t U8_Buff[4];
    float Float;
} Bint32_Union;

typedef struct
{
    bool receiveStart;                              // 数据接收开始标志
    uint8_t receiveIndex;                           // 接收序列号
    bool receiveFinished;                           // 接收并校验成功
    uint8_t receiveBuff[UsbFrameLengthMax];         // 临时接收数据区
    uint8_t receiveBuffFinished[UsbFrameLengthMax]; // 校验完成数据区
} Usb_Struct;

class Driver
{

private:
    std::shared_ptr<SerialPort> _serial_port = nullptr;
    std::string _port_name; // 端口名字
    BaudRate _bps;          // 波特率
    bool isOpen = false;
    Usb_Struct usb_Struct;

private:
    int recv(unsigned char &charBuffer, size_t msTimeout = 0)
    {
        /*try检测语句块有没有异常。如果没有发生异常,就检测不到。
        如果发生异常，則交给 catch 处理，执行 catch 中的语句* */
        try
        {
            /*从串口读取一个数据,指定msTimeout时长内,没有收到数据，抛出异常。
            如果msTimeout为0，则该方法将阻塞，直到数据可用为止。*/
            _serial_port->ReadByte(charBuffer, msTimeout); // 可能出现异常的代码段
        }
        catch (const ReadTimeout &) // catch捕获并处理 try 检测到的异常。
        {
            // std::cerr << "The ReadByte() call has timed out." << std::endl;
            return -2;
        }
        catch (const NotOpen &) // catch()中指明了当前 catch 可以处理的异常类型
        {
            std::cerr << "Port Not Open ..." << std::endl;
            return -1;
        }
        return 0;
    };

    int send(unsigned char charbuffer)
    {

        // try检测语句块有没有异常
        try
        {
            _serial_port->WriteByte(charbuffer); // 写数据到串口
        }
        catch (const std::runtime_error &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "The Write() runtime_error." << std::endl;
            return -2;
        }
        catch (const NotOpen &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Port Not Open ..." << std::endl;
            return -1;
        }
        _serial_port->DrainWriteBuffer(); // 等待，直到写缓冲区耗尽，然后返回。
        return 0;
    }

public:
    // 定义构造函数
    Driver(const std::string &port_name, BaudRate bps) : _port_name(port_name), _bps(bps){};
    // 定义析构函数
    ~Driver() { close(); };

public:
    int open()
    {
        _serial_port = std::make_shared<SerialPort>();
        if (_serial_port == nullptr)
        {
            std::cerr << "Serial Create Failed ." << std::endl;
            return -1;
        }
        // try检测语句块有没有异常
        try
        {
            _serial_port->Open(_port_name);                               // 打开串口
            _serial_port->SetBaudRate(_bps);                              // 设置波特率
            _serial_port->SetCharacterSize(CharacterSize::CHAR_SIZE_8);   // 8位数据位
            _serial_port->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // 设置流控
            _serial_port->SetParity(Parity::PARITY_NONE);                 // 无校验
            _serial_port->SetStopBits(StopBits::STOP_BITS_1);             // 1个停止位
        }
        catch (const OpenFailed &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << _port_name << "open failed ..."
                      << std::endl;

            isOpen = false;
            return -2;
        }
        catch (const AlreadyOpen &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << _port_name << "open failed ..."
                      << std::endl;
            isOpen = false;
            return -3;
        }
        catch (...) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << _port_name << " recv exception ..."
                      << std::endl;
            isOpen = false;
            return -4;
        }

        usb_Struct.receiveStart = false;
        usb_Struct.receiveIndex = 0;
        usb_Struct.receiveFinished = false;

        isOpen = true;
        return 0;
    };

    int senddata(unsigned char charbuffer) { return send(charbuffer); }
    int recvdata(unsigned char &charBuffer, size_t msTimeout) { return recv(charBuffer, msTimeout); }

    /**
     * @brief 智能车速度与方向控制
     *
     * @param speedset 速度单位：m/s
     * @param servoPwm 舵机方向：500~2500/PWM
     */
    void carControl(int speed, int midAdd, int buzzer) {
        // buzzer = 0;
        if (isOpen) {
            // unsigned char sendData[8];
            // sendData[0] = 0x23;//帧头
            // sendData[1] = 0x00;//舵机模式 0x00 接受误差值; 0x01~ 其他模式
            // sendData[2] = (unsigned char)midAdd;//舵机误差值
            // sendData[3] = speed;//电机模式 0x00 停 0x01~ 其他模式0x02慢速0x01快速
            // sendData[4] = 0x00;//蜂鸣器模式 0x00 关;0x01 开
            // sendData[5] = 0x05;
            // sendData[6] = 0x06;
            // sendData[7] = 0x21;//帧尾
            // buzzer = 0;
            if(buzzer != 0){
                buzzer = 3;
            }
            unsigned char sendData[8];
            extern float speed_set;
            extern int rescue_state;
            // extern SPEED_MODE speed_mode;
            extern int speed_pid_mode;
            speed_set = speed;
            sendData[0] = 0xff; //帧头
            sendData[1] = speed_pid_mode;
            /*speed_mode
            * enum {
            *  SPEED_NONE = 0,
            *  NORMAL,
            *  BANGBANG,       //BANGBANG
            *  SOFT            //SOFT
            * };
            */
            sendData[2] = rescue_state;
            sendData[3] = (unsigned char)speed;
            sendData[4] = (unsigned char)buzzer;//0x00;//蜂鸣器模式 0x00 关;0x01 开
            sendData[5] = midAdd/100;
            sendData[6] = midAdd%100;
            sendData[7] = 0xfe;//帧尾
            // std::cout<<"speed: "<<(int)speed<<" rescue_state: "<<(int)rescue_state<<std::endl;
            std::cout<<"speed: "<<(int)speed<<std::endl;
            // 循环发送数据
            for (size_t i = 0; i < 8; i++) {
                send(sendData[i]);
            }
        }
        else
        {
            std::cout << "Error: Uart Open failed!!!!" << std::endl;
        }
    }

    /**
     * @brief 蜂鸣器音效
     *
     * @param sound
     * >  1：确认/OK
     * >  2：报警/Warnning
     * >  3：完成/Finish
     * >  4：提示/Ding
     * >  5：开机/Systemstart
     */
    void buzzerSound(unsigned char sound)
    {
        if (isOpen)
        {
            unsigned char sendBuff[7];
            unsigned char check = 0;

            sendBuff[0] = 0x42;  // 帧头
            sendBuff[1] = 0x04;  // 地址
            sendBuff[2] = 5;     // 帧长
            sendBuff[3] = sound; // 音效

            for (size_t i = 0; i < 4; i++)
            {
                check += sendBuff[i];
            }
            sendBuff[4] = check;

            // 循环发送数据
            for (size_t i = 0; i < 7; i++)
            {
                send(sendBuff[i]);
            }
        }
        else
        {
            std::cout << "Error: Uart Open failed!!!!" << std::endl;
        }
    }

    /**
     * @brief 串口接收下位机比赛开始信号
     *
     */
    bool receiveStartSignal(void)
    {
        uint8_t resByte;
        int ret = 0;

        ret = recvdata(resByte, 3000);
        if (ret == 0)
        {
            if (resByte == UsbFrameHead && !usb_Struct.receiveStart) // 帧头检测
            {
                usb_Struct.receiveStart = true;
                usb_Struct.receiveBuff[0] = resByte;
                usb_Struct.receiveBuff[2] = UsbFrameLengthMin;
                usb_Struct.receiveIndex = 1;
            }
            else if (usb_Struct.receiveIndex == 2) // 数据长度
            {
                usb_Struct.receiveBuff[usb_Struct.receiveIndex] = resByte;
                usb_Struct.receiveIndex++;

                if (resByte > UsbFrameLengthMax || resByte < UsbFrameLengthMin) // 帧长校验
                {
                    usb_Struct.receiveBuff[2] = UsbFrameLengthMin;
                    usb_Struct.receiveIndex = 0;
                    usb_Struct.receiveStart = false;
                }
            }
            else if (usb_Struct.receiveStart && usb_Struct.receiveIndex < UsbFrameLengthMax)
            {
                usb_Struct.receiveBuff[usb_Struct.receiveIndex] = resByte;
                usb_Struct.receiveIndex++;
            }

            // 帧接收完毕
            if ((usb_Struct.receiveIndex >= UsbFrameLengthMax || usb_Struct.receiveIndex >= usb_Struct.receiveBuff[2]) && usb_Struct.receiveIndex > UsbFrameLengthMin)
            {
                uint8_t check = 0;
                uint8_t length = UsbFrameLengthMin;

                length = usb_Struct.receiveBuff[2];
                for (int i = 0; i < length - 1; i++)
                    check += usb_Struct.receiveBuff[i];

                if (check == usb_Struct.receiveBuff[length - 1]) // 校验位
                {
                    memcpy(usb_Struct.receiveBuffFinished, usb_Struct.receiveBuff, UsbFrameLengthMax);
                    usb_Struct.receiveFinished = true;

                    // 任务开始指令
                    if (0x06 == usb_Struct.receiveBuffFinished[1])
                    {
                        return true;
                    }
                }

                usb_Struct.receiveIndex = 0;
                usb_Struct.receiveStart = false;
            }
        }

        return false;
    }

    void close()
    {
        if (_serial_port != nullptr)
        {
            /*关闭串口。串口的所有设置将会丢失，并且不能在串口上执行更多的I/O操作。*/
            _serial_port->Close();
            _serial_port = nullptr;
        }
    };
};
