#include "Main.hpp"


std::chrono::time_point<std::chrono::system_clock> time_since_move, begin_time, end_time;

modbus_t *ctx_switch;
modbus_t *ctx;
modbus_t *ctx_hmi;

uint16_t _3x[128];
uint16_t _4x[128];

uint16_t limit_switches;
bool status;
bool prev_status;

int ppi = 312;
int smoothing[100];
int head = 0;
int base = 7;
int prev_shift = 0;

Images images;

int tsmooth[100];
int thead = 0;
int tbase = 35;
int cspeed(){
    tsmooth[thead] = images.travel;

    thead = (thead+1)%tbase;

    int speed = 0;
    for(int i =0; i<tbase; i++){
        speed += tsmooth[i];
    }
    speed = speed / tbase;
    return abs(speed);
}

float toFloat(int m, int reg){
    reg--;
    int32_t pl;
    if (m == 3){
        pl = ((uint32_t)_3x[reg+1] << 16) + (uint16_t)_3x[reg];
    }
    else if (m == 4){
        pl = ((uint32_t)_4x[reg+1] << 16) + (uint16_t)_4x[reg];
    }
    return (float)pl * 0.01;
}

void toUint(int reg, float var){
    var = var*100;
    uint16_t a = ((int16_t)var >> 16);
    uint16_t b = ((int16_t)var);

    _4x[reg] = a;
    _4x[reg-1] = b;
}

void changeBit(int bit, int reg, int val){
    std::bitset<16> servo_command_word(_4x[reg-1]);
    servo_command_word[bit] = val;
    _4x[reg-1] = servo_command_word.to_ulong();
}

void configure_servo(){
    modbus_read_input_registers(ctx, 0, 54, _3x);
    modbus_read_registers(ctx, 0, 54, _4x);

    toUint(21,40.0); // accel
    toUint(25,40.0); // decel
    toUint(29,2.0); // speed
    _4x[48] = 2;
    _4x[51] = 1;

    changeBit(2,1,1);
    changeBit(0,1,0);

    _4x[49] = 1; // homing type
    _4x[18] = _3x[48]; // ...
    _4x[19] = _3x[49]; // home position to current position

    modbus_write_registers(ctx, 0, 54, _4x);
}

void startServo(){
    changeBit(2,1,0); // stop homing

    std::bitset<16> servo_command_word_1(_4x[0]);
    servo_command_word_1[0] = 0;
    _4x[0] = servo_command_word_1.to_ulong();
    modbus_write_registers(ctx, 0, 1, &_4x[0]);

    modbus_write_registers(ctx, 2, 1, _4x);

    time_since_move = std::chrono::system_clock::now();
}

void reloadVision(){
    images.pattern_image = images.current_image.clone();
    cv::imwrite("Pattern_new.Bmp", images.pattern_image);
}

int get_new_image (CameraPtr pCam){
    try
    {
        ImagePtr convertedImage = pCam->GetNextImage(120);
        if (convertedImage->IsIncomplete())
        {
            //std::cout << "Image incomplete..." << std::endl;
            return -1;
        }
        else{
            unsigned int XPadding = convertedImage->GetXPadding();
            unsigned int YPadding = convertedImage->GetYPadding();
            unsigned int rowsize  = convertedImage->GetWidth();
            unsigned int colsize  = convertedImage->GetHeight();

            cv::Mat sample( colsize+YPadding, 
                            rowsize+XPadding, 
                            CV_8UC3, 
                            convertedImage->GetData(), 
                            convertedImage->GetStride());

            images.current_image = sample.clone();
            images.c_stamp = std::chrono::system_clock::now();
        }
        convertedImage->Release();

    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

void resetAverage(){
    for(int i =0; i<base; i++){
        smoothing[i] = 0;
    }
}

float deviation(){
    /*
    if (abs(images.shift-prev_shift) < 100){
        smoothing[head] = images.shift;
        prev_shift = images.shift;
    }
    else{
        smoothing[head] = prev_shift;
    }
    */

    smoothing[head] = images.shift;

    head = (head+1)%base;

    float deviation = 0;
    for(int i =0; i<base; i++){
        deviation += smoothing[i];
    }
    deviation = deviation / base;

    deviation = deviation / ppi;

    return deviation;// + 0.125;
}

int main()
{
    /*
    ctx_hmi = modbus_new_tcp("192.168.1.45", 502);
    if (modbus_connect(ctx_hmi) != 0){
        std::cout << "hmi connection failure" << std::endl;
        return -1;
    }
    */
    ctx_switch = modbus_new_tcp("192.168.1.100", 502);
    if ( modbus_connect(ctx_switch) != 0){
        std::cout << "limit switch connection failure" << std::endl;
        return -1;
    }

    ctx = modbus_new_tcp(servo_address, 502);
    if ( modbus_connect(ctx) != 0){
        std::cout << "servo connection failure" << std::endl;
        return -1;
    }

    configure_servo();

    images.pattern_image = cv::imread("pattern.Bmp");

    std::ifstream infile("serial_number.txt");
    std::string serial_number = "";
    infile >> serial_number;
    infile.close();

    SystemPtr system = System::GetInstance();
    CameraList camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();

    if (numCameras == 0)
    {
        camList.Clear();
        system->ReleaseInstance();
        modbus_free(ctx);
        std::cout << "no cameras" << std::endl;
        return -1;
    }

    CameraPtr pCam = nullptr;
    int result = 0;
    pCam = camList.GetBySerial(serial_number);

    if (pCam == nullptr){
        modbus_free(ctx);
        std::cout << "camera connection failure" << std::endl;
        return -1;
    }

    pCam->Init();
    INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
    INodeMap& nodeMap = pCam->GetNodeMap();

    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    ptrAcquisitionMode->SetIntValue(ptrAcquisitionModeContinuous->GetValue());
    pCam->BeginAcquisition();
    std::cout << "acquiring" << std::endl;

    get_new_image(pCam);
    get_new_image(pCam);
    prev_shift = images.shift;

    startServo();

    while (true){
        begin_time = std::chrono::system_clock::now(); 

        modbus_read_input_registers(ctx, 0, 82, _3x);

        std::bitset<16> servo_status_word_1(_3x[44]);
        std::bitset<16> servo_status_word_2(_3x[45]);
        std::bitset<16> servo_status_word_3(_3x[46]);
        std::bitset<16> servo_status_word_4(_3x[47]);
        std::bitset<16> servo_command_word_1(_4x[0]);
        std::bitset<16> servo_command_word_3(_4x[2]);

        if (status != prev_status && status == true){
            reloadVision();
            servo_command_word_1[0] = 1;
            resetAverage();
        }
        else if (status == false){
            servo_command_word_1[0] = 0;   
        }
        
        bool new_image = -1;
        while(new_image != 0){
            new_image = get_new_image(pCam);
        }
        getMovement(&images);

        float servo_position = toFloat(3,49);
        float old_command = toFloat(4,37);
        float dev = deviation();

        //std::cout << servo_status_word_1 << "\t";
        //std::cout << servo_status_word_2 << "\t";
        //std::cout << servo_status_word_3 << "\t";
        //std::cout << servo_status_word_4 << "\t";
        //std::cout << servo_command_word_1 << "\t";
        //std::cout << servo_command_word_3 << "\t";
        //std::cout << old_command << " " << servo_position << " " << dev << " " << images.shift;
        //std::cout << cspeed();
        //std::cout << std::endl;

        std::chrono::milliseconds dwell_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-time_since_move);
        if (dwell_time.count() > 30 && dwell_time.count() < 90){
            servo_command_word_1[3] = 0;
            _4x[0] = servo_command_word_1.to_ulong();
            modbus_write_registers(ctx, 0, 54, _4x);
        }
        else if (dwell_time.count() >= 100){
            time_since_move = std::chrono::system_clock::now(); 

            if (abs(dev)> 0.03 && status && servo_status_word_1[4]){
                servo_command_word_1[3] = 1;
            }
            float servo_position_command = servo_position + dev;
            toUint(37,servo_position_command);

            modbus_read_registers(ctx_switch, 31, 1, &limit_switches);
            std::bitset<16> stride_word(limit_switches);
            prev_status = status;
            status = stride_word[module_n-1];

            servo_command_word_3[0] = servo_status_word_1[0]; // heartbeat
            _4x[0] = servo_command_word_1.to_ulong();
            _4x[2] = servo_command_word_3.to_ulong();
            modbus_write_registers(ctx, 0, 54, _4x);

        }

        end_time = std::chrono::system_clock::now();
        std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-begin_time);     
        //std::cout << elapsed.count() << std::endl;
    }

    // Deinitialize camera
    pCam->EndAcquisition();
    pCam->DeInit();
    pCam = nullptr;
    camList.Clear();
    system->ReleaseInstance();
    ////////////////////////////////////////////////////////////////

    // do not exit the program until communications have shutdown
    std::cout << "Camera shutdown: Success" << std::endl;

    modbus_free(ctx);
    modbus_free(ctx_switch);
}
