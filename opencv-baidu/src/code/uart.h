#ifndef _WIN32
#include <iostream>
#include <libserial/SerialPort.h>
#include <string.h>

using namespace LibSerial;

#define UsbFrameHead 0x42    // USBͨ��֡ͷ
#define UsbFrameLengthMin 4  // USBͨ��֡��̳��ȣ��ֽڣ�
#define UsbFrameLengthMax 30 // USBͨ��֡����ȣ��ֽڣ�

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
    bool receiveStart;                              // ���ݽ��տ�ʼ��־
    uint8_t receiveIndex;                           // �������к�
    bool receiveFinished;                           // ���ղ�У��ɹ�
    uint8_t receiveBuff[UsbFrameLengthMax];         // ��ʱ����������
    uint8_t receiveBuffFinished[UsbFrameLengthMax]; // У�����������
} Usb_Struct;

class Driver
{

private:
    std::shared_ptr<SerialPort> _serial_port = nullptr;
    std::string _port_name; // �˿�����
    BaudRate _bps;          // ������
    bool isOpen = false;
    Usb_Struct usb_Struct;

private:
    int recv(unsigned char& charBuffer, size_t msTimeout = 0)
    {
        /*try���������û���쳣�����û�з����쳣,�ͼ�ⲻ����
        ��������쳣���t���� catch ����ִ�� catch �е����* */
        try
        {
            /*�Ӵ��ڶ�ȡһ������,ָ��msTimeoutʱ����,û���յ����ݣ��׳��쳣��
            ���msTimeoutΪ0����÷�����������ֱ�����ݿ���Ϊֹ��*/
            _serial_port->ReadByte(charBuffer, msTimeout); // ���ܳ����쳣�Ĵ����
        }
        catch (const ReadTimeout&) // catch���񲢴��� try ��⵽���쳣��
        {
            // std::cerr << "The ReadByte() call has timed out." << std::endl;
            return -2;
        }
        catch (const NotOpen&) // catch()��ָ���˵�ǰ catch ���Դ�����쳣����
        {
            std::cerr << "Port Not Open ..." << std::endl;
            return -1;
        }
        return 0;
    };

    int send(unsigned char charbuffer)
    {

        // try���������û���쳣
        try
        {
            _serial_port->WriteByte(charbuffer); // д���ݵ�����
        }
        catch (const std::runtime_error&) // catch���񲢴��� try ��⵽���쳣��
        {
            std::cerr << "The Write() runtime_error." << std::endl;
            return -2;
        }
        catch (const NotOpen&) // catch���񲢴��� try ��⵽���쳣��
        {
            std::cerr << "Port Not Open ..." << std::endl;
            return -1;
        }
        _serial_port->DrainWriteBuffer(); // �ȴ���ֱ��д�������ľ���Ȼ�󷵻ء�
        return 0;
    }

public:
    // ���幹�캯��
    Driver(const std::string& port_name, BaudRate bps) : _port_name(port_name), _bps(bps) {};
    // ������������
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
        // try���������û���쳣
        try
        {
            _serial_port->Open(_port_name);                               // �򿪴���
            _serial_port->SetBaudRate(_bps);                              // ���ò�����
            _serial_port->SetCharacterSize(CharacterSize::CHAR_SIZE_8);   // 8λ����λ
            _serial_port->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // ��������
            _serial_port->SetParity(Parity::PARITY_NONE);                 // ��У��
            _serial_port->SetStopBits(StopBits::STOP_BITS_1);             // 1��ֹͣλ
        }
        catch (const OpenFailed&) // catch���񲢴��� try ��⵽���쳣��
        {
            std::cerr << "Serial port: " << _port_name << "open failed ..."
                << std::endl;

            isOpen = false;
            return -2;
        }
        catch (const AlreadyOpen&) // catch���񲢴��� try ��⵽���쳣��
        {
            std::cerr << "Serial port: " << _port_name << "open failed ..."
                << std::endl;
            isOpen = false;
            return -3;
        }
        catch (...) // catch���񲢴��� try ��⵽���쳣��
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
    int recvdata(unsigned char& charBuffer, size_t msTimeout) { return recv(charBuffer, msTimeout); }

    /**
     * @brief ���ܳ��ٶ��뷽�����
     *
     * @param speed �ٶȵ�λ��m/s
     * @param servoPwm �������500~2500/PWM
     */
    void carControl(float speed, uint16_t servoPwm)
    {
        if (isOpen)
        {
            Bint32_Union bint32_Union;
            Bint16_Union bint16_Union;
            unsigned char sendBuff[12];
            unsigned char check = 0;

            bint32_Union.Float = speed;

            if (servoPwm > PWMSERVOMAX)
                servoPwm = PWMSERVOMAX;
            else if (servoPwm < PWMSERVOMIN)
                servoPwm = PWMSERVOMIN;
            bint16_Union.U16 = servoPwm;

            sendBuff[0] = 0x42; // ֡ͷ
            sendBuff[1] = 0x01; // ��ַ
            sendBuff[2] = 10;   // ֡��

            sendBuff[3] = bint32_Union.U8_Buff[0]; // �ٶ�
            sendBuff[4] = bint32_Union.U8_Buff[1];
            sendBuff[5] = bint32_Union.U8_Buff[2];
            sendBuff[6] = bint32_Union.U8_Buff[3];

            sendBuff[7] = bint16_Union.U8_Buff[0]; // ����
            sendBuff[8] = bint16_Union.U8_Buff[1];

            for (size_t i = 0; i < 9; i++)
            {
                check += sendBuff[i];
            }
            sendBuff[9] = check;

            // ѭ����������
            for (size_t i = 0; i < 10; i++)
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
     * @brief ��������Ч
     *
     * @param sound
     * >  1��ȷ��/OK
     * >  2������/Warnning
     * >  3�����/Finish
     * >  4����ʾ/Ding
     * >  5������/Systemstart
     */
    void buzzerSound(unsigned char sound)
    {
        if (isOpen)
        {
            unsigned char sendBuff[7];
            unsigned char check = 0;

            sendBuff[0] = 0x42;  // ֡ͷ
            sendBuff[1] = 0x04;  // ��ַ
            sendBuff[2] = 5;     // ֡��
            sendBuff[3] = sound; // ��Ч

            for (size_t i = 0; i < 4; i++)
            {
                check += sendBuff[i];
            }
            sendBuff[4] = check;

            // ѭ����������
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
     * @brief ���ڽ�����λ��������ʼ�ź�
     *
     */
    bool receiveStartSignal(void)
    {
        uint8_t resByte;
        int ret = 0;

        ret = recvdata(resByte, 3000);
        if (ret == 0)
        {
            if (resByte == UsbFrameHead && !usb_Struct.receiveStart) // ֡ͷ���
            {
                usb_Struct.receiveStart = true;
                usb_Struct.receiveBuff[0] = resByte;
                usb_Struct.receiveBuff[2] = UsbFrameLengthMin;
                usb_Struct.receiveIndex = 1;
            }
            else if (usb_Struct.receiveIndex == 2) // ���ݳ���
            {
                usb_Struct.receiveBuff[usb_Struct.receiveIndex] = resByte;
                usb_Struct.receiveIndex++;

                if (resByte > UsbFrameLengthMax || resByte < UsbFrameLengthMin) // ֡��У��
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

            // ֡�������
            if ((usb_Struct.receiveIndex >= UsbFrameLengthMax || usb_Struct.receiveIndex >= usb_Struct.receiveBuff[2]) && usb_Struct.receiveIndex > UsbFrameLengthMin)
            {
                uint8_t check = 0;
                uint8_t length = UsbFrameLengthMin;

                length = usb_Struct.receiveBuff[2];
                for (int i = 0; i < length - 1; i++)
                    check += usb_Struct.receiveBuff[i];

                if (check == usb_Struct.receiveBuff[length - 1]) // У��λ
                {
                    memcpy(usb_Struct.receiveBuffFinished, usb_Struct.receiveBuff, UsbFrameLengthMax);
                    usb_Struct.receiveFinished = true;

                    // ����ʼָ��
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
            /*�رմ��ڡ����ڵ��������ý��ᶪʧ�����Ҳ����ڴ�����ִ�и����I/O������*/
            _serial_port->Close();
            _serial_port = nullptr;
        }
    };
};

#endif // !_WIN32