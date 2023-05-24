#ifndef SYSTEMINTERFACE_H
#define SYSTEMINTERFACE_H
#include <cstdint>

inline uint32_t swap_endian32(uint32_t num)
{
    return ((num << 24) & 0xff000000) | ((num << 8) & 0xff0000) | ((num >> 8) & 0xff00) | ((num >> 24) & 0xff);
}

inline uint16_t swap_endian16(uint16_t value)
{
    return (value << 8) | (value >> 8);
}

#pragma pack(1)

enum class MsgID
{
    //此处为大端
    Visiblelight = 0x2101,
    Infrared = 0x2102,
    OtherLight = 0x2103,
    Y9SAR = 0x2104,
    OtherSAR = 0x2105
    // Visiblelight = 0x0121,
    // Infrared = 0x0221,
    // OtherLight = 0x0321,
    // Y9SAR = 0x0421,
    // OtherSAR = 0x0521
};

struct PkgHeader
{
    uint32_t header = 0x55555555; // 固定值：0x55555555
    uint32_t length;              // 包括从帧头到帧尾的所有字段字节长度
    uint16_t srcId;               // 消息发送的逻辑ID
    uint16_t dstId;               // 消息接收的逻辑ID
    uint8_t type = 0x02;          // 消息类型：0x02
    uint8_t msgType = 0xF1;       // 消息类别：0xF1
    uint16_t id = 0x2101;         // 消息ID：0x2101
    uint64_t sendTime;            // 参见时间定义规则
    uint16_t sendSeq;             // 表明上电后该条消息发送次，从1累加，越界后继续从1开始累加
    uint8_t reserved[6] = {0};    // 默认0
};
struct PkgTail
{
    uint32_t tail = 0xAAAAAAAA; // 固定值：0xAAAAAAAA
};

struct VisibleLightInfrareMsg
{
    PkgHeader head;
    uint8_t command[20] = {0};      // 命令编号
    uint16_t totalPackageNum;       // 图像数据的总包数，取值范围：[]
    uint8_t subPackageSeq[4] = {0}; // 子包序号，取值范围：1~0xFFFFFF
    uint16_t imageRows;             // 单张图像的行数，取值范围[0，4095]
    uint16_t imageCols;             // 单张图像的列数，取值范围[0，4095]
    uint8_t quantization; // 量化位数：0x01表示8bit，0x02表示16bit    uint8_t reserved2 = 0; // 默认
    uint8_t reserved2;    // 默认0
    uint32_t dataLength;  // 有效数据长度N，取值范围：[0~256K]
    // uint8_t img_msg[dataLength];          // 图像数据
};

struct OthetLightMsg
{
    PkgHeader head;
    uint8_t command_id[20] = {0};  // 11. 命令编号
    uint8_t image_name[256] = {0}; // 12. 图像名称
    uint8_t image_type;            // 13. 图像类型
    uint8_t image_source;          // 14. 图像来源
    uint16_t image_resolution;     // 15. 图像分辨率
    uint32_t image_row_count;      // 16. 图像行数
    uint32_t image_column_count;   // 17. 图像列数
    uint8_t quantization_bits;     // 18. 量化位数
    uint16_t total_packets;        // 19. 图像数据的总包数
    uint32_t packet_index;         // 20. 子包序号
    uint8_t reserved;              // 21. 保留
    uint32_t data_length;          // 22. 有效数据长度N
    // std::vector<uint8_t> data;   // 23. 数据信息
};

struct PositionInformationOfImageFeaturePoints
{
    uint32_t REP;  // 重复因子，标识图像特征点个数 描述：标识图像特征点的个数
    uint32_t lon1; // 特征点1经度，单位：°，精度：180°/2E31 描述：正数表示东经，负数标识西经，取值范围（-180,~180）°
    uint32_t lat1; // 特征点1纬度，单位：°，精度：90°/2E31 描述：正数表示北纬，负数表示南纬，取值范围（-90,~90）°
    uint32_t alt1; // 特征点1高度，单位：米，精度：1 描述：海拔高度，取值范围[-460,~30000）°
    uint32_t lon2; // 特征点2经度，单位：°，精度：180°/2E31 描述：正数表示东经，负数标识西经，取值范围（-180,~180）°
    uint32_t lat2; // 特征点2纬度，单位：°，精度：90°/2E31 描述：正数表示北纬，负数表示南纬，取值范围（-90,~90）°
    uint32_t alt2; // 特征点2高度，单位：米，精度：1 描述：海拔高度，取值范围[-460,~30000）°
    uint32_t lon3; // 特征点3经度，单位：°，精度：180°/2E31 描述：正数表示东经，负数标识西经，取值范围（-180,~180）°
    uint32_t lat3; // 特征点3纬度，单位：°，精度：90°/2E31 描述：正数表示北纬，负数表示南纬，取值范围（-90,~90）°
    uint32_t alt3; // 特征点3高度，单位：米，精度：1 描述：海拔高度，取值范围[-460,~30000）°
};

// 定义消息帧头部分的结构体
struct Y9SARMsg1
{
    PkgHeader head;
    char commandNumber[20] = {0};        // 命令编号
    char sensorActivityNumber[40] = {0}; // 传感器活动编号
    uint16_t antennaType = 0x06;         // 天线类型，本线程固定为06
    uint16_t antennaCategory = 0x01;     // 天线类别，本线程固定为01
    uint8_t reserved1[89] = {0};         // 保留字段，长度为89
    uint64_t imagingStartTime;           // 成像起始时间，参见时间定义规则
    uint64_t imagingEndTime;             // 成像结束时间，参见时间定义规则
    uint8_t reserved2[14] = {0};         // 保留字段，长度为14
    uint32_t dataLength;                 // 有效数据包长度，有效数据包长度：≤1M字节
    uint8_t reserved3[20] = {0};         // 保留字段，长度为20
    uint32_t taskNumber;                 // 任务代号，任务规划系统规划任务时确定，保留
    uint32_t aircraftNumber;             // 飞机编号，保留
    uint8_t reserved4[2] = {0};          // 保留字段，长度为2
    uint16_t
        bootCount; // 开机次数，对SAR雷达，一次飞行过程中切换模式的序号，每切换一次工作模式，开机次数需递增。从0开始。
    uint8_t reserved5[106] = {0}; // 保留字段，长度为106
    uint16_t frameNumber; // 帧编号，雷达扫描周期计数，模式切换时帧编号计数器清零，从0开始计数，最大65535。
    uint16_t packetCount;  // 数据包总数，对一帧图像进行分包传输，分包的总包数
    uint16_t packetNumber; // 数据包序号，一次处理周期内产生的数据包的序号，序号从0计数，最大取值为“数据包总数-1”
    uint8_t reserved6[64] = {0}; // 保留字段，长度为64
    uint16_t resolution; // 分辨率，单位：0.01m，值域：0.1m，0.15m，0.3m，0.5m，1m，3m，5m，10m，50m，100m
    uint8_t reserved7[18] = {0}; // 保留字段，长度为18
    uint32_t
        imageHeight; // 图像高H，即方位向，方位向定义为平台运动方向投影到大地坐标系水平面上的方向，也即沿载机航迹，正交于波束指向，单位为点。
    uint32_t
        imageWidth; // 图像宽W，即距离向，距离向定义为雷达波束指向投影到大地坐标系水平面上的方向，也即沿波束指向，图像条幅宽度，单位为点。
    float azimuthPixelSpacing; // 方位向像素间隔，方位向定义为平台运动方向投影到大地坐标系水平面上的方向，单位为m
    float rangePixelSpacing; // 距离向像素间隔，距离向定义为雷达波束指向投影到大地坐标系水平面上的方向，单位为m
    PositionInformationOfImageFeaturePoints
        imageFeaturePosition; // 图像特征点位置，为一幅图像特征点的位置信息，参见表10
    uint32_t imageSize;       // 图像像素个数M，一包图像数据的大小（包长度）：≤1M字节
};
struct Y9SARMsg2
{
    // struct uint8_t *imageData;                        // 图像像素数据，像素值
    uint32_t quantizationBits;                        // 图像量化位数，1-8bit量化其它保留
    uint8_t imageType;                                // 图像类型，0x01：斜距；0x02：地距；其它保留
    float reserved8;                                  // 保留字段，长度为4
    uint8_t packetTail[4] = {0x7E, 0x7E, 0x7E, 0x7E}; // 数据包尾，固定值：0x7E7E7E7E
    PkgTail tail;
};

struct OtherSARMsg1
{
    PkgHeader head;
    uint8_t commandNumber[20] = {0}; // 命令编号
    uint8_t imageName[256] = {0};    // 图像名称，字符串路径
    uint8_t imageSource; // 图像来源：0x01-卫星平台，0x02-机载平台，0x03-无人机平台，其他保留
    uint8_t workMode;    // 工作模式：0x01-条带，0x02-聚束，其他保留
    uint16_t imageResolution;  // 图像分辨率：单位-米，LSB-0.01
    uint32_t imageRowCount;    // 图像行数：单张图像的行数，沿平台航迹方向，正交于波束指向
    uint32_t imageColumnCount; // 图像列数：单张图像的列数，沿波束指向，图像条幅宽度
    uint8_t quantizationBit;   // 量化位数：0x01-8bit，其他保留
    PositionInformationOfImageFeaturePoints
        imageFeaturePosition;         // 图像特征点位置，为一幅图像特征点的位置信息，参见表10
    uint16_t totalPacketCount;        // 图像数据的总包数：取值范围：[]
    uint8_t subPacketNumber[4] = {0}; // 子包序号：取值范围；1~0xFFFFFF
    uint32_t validDataLength;         // 有效数据长度N：取值范围：[0~256K]
};
struct OtherSARMsg2
{
    // struct uint8_t *imageData;                        // 图像像素数据，像素值
    uint8_t reserved[2]; // 保留字段，长度为4
    PkgTail tail;
};

struct ImageDetectionResult
{
    struct TargetMsg
    {
        uint8_t image_storage_path[256]; // 目标图像存储路径，存入FTP，各家约定默认存储路径，该字段填写图像名称，如1.BMP
        uint8_t param_file_storage_path[256] = {
            0}; // 图像参数文件存储路径，存入FTP，各家约定默认存储路径，该字段填写图像名称，如1.txt
        uint8_t image_source; // 图像来源，0x01：Y9，0x02：无侦-7，0x03：卫星，0x04：无人机，其他预留
        uint32_t target_image_corner_pixel_coordinate[8] = {
            0}; // 目标图像四角像素坐标，图像坐标系为左上角为原点，x轴向右，y轴向下，顺时针方向依次为：a[0]：左上角x坐标，a[1]：左上角y坐标，a[2]：右上角x坐标，a[3]：右上角y坐标，a[4]：右下角x坐标，a[5]：右下角y坐标，a[6]：左下角x坐标，a[7]：左下角y坐标，单位：像素，LSB
                // // // // = 0.01
        uint32_t target_image_center_pixel_coordinate
            [2]; // 目标图像中心点像素坐标，图像坐标系为左上角为原点，x轴向右，y轴向下：a[0]：中心点x坐标，a[1]：中心点y坐标，单位：像素，LSB
                 // = 0.01
        uint8_t target_locating_method_1_validity; // 目标定位方式1有效状态，传感器直接定位，0x01：有效，0x02：无效
        uint8_t target_locating_method_2_validity; // 目标定位方式2有效状态，底图匹配定位，0x01：有效，0x02：无效
        uint8_t target_locating_method_3_validity; // 目标定位方式3有效状态，光SAR匹配定位，0x01：有效，0x02：无效
        int32_t longitude_1; // 方式1（传感器直接定位）定位结果，单位：度，LSB：0.000001，对应值域[-180, 180]
        int32_t latitude_1; // 方式1（传感器直接定位）定位结果，单位：度，LSB：0.000001，对应值域[-90, 90]
        int32_t height_1; // 方式1（传感器直接定位）定位结果，单位：米，LSB：0.001
        int32_t longitude_2; // 方式2（底图匹配定位）定位结果，单位：度，LSB：0.000001，对应值域[-180, 180]
        int32_t latitude_2; // 方式2（底图匹配定位）定位结果，单位：度，LSB：0.000001，对应值域[-90, 90]
        int32_t height_2; // 方式2（底图匹配定位）定位结果，单位：米，LSB：0.001
        int32_t longitude_3; // 方式3（光SAR匹配定位）定位结果，单位：度，LSB：0.000001，对应值域[-180, 180]
        int32_t latitude_3; // 方式3（光SAR匹配定位）定位结果，单位：度，LSB：0.000001，对应值域[-90, 90]
        int32_t height_3; // 方式3（光SAR匹配定位）定位结果，单位：米，LSB：0.001
    };
    PkgHeader header;          // 包头
    uint8_t cmd_id[20] = {0};  // 命令编号
    uint64_t recv_timestamp;   // 接收时间戳
    uint64_t finish_timestamp; // 处理完成时间戳
    uint8_t
        target_composition; // 目标图像组成，bit0：表示可见光；0-无，1-有；bit1：表示红外；0-无，1-有；bit2：表示SAR；0-无，1-有；其他预留
    uint8_t target_id[16] = {0}; // 目标编号，参见目标编号命名规则
    uint8_t
        target_source; // 目标来源，默认：0000 // // //
                       // 0000；bit0：广域成像WZ智能检测单元；0-无，1-有；bit1：弱特征图像目标智能识别单元；0-无理，1-有；bit2：倾斜检测目标智能定位原型软件，0-无；1-有；其它预留
    uint8_t
        targetType; // 目标类型 UINT8 1 // // //
                    // 0x01：车辆类，0x02：飞机类，0x03：舰船类，0x11：小型车（坦克），0x12：中型车（军用方舱车），0x13：其他车辆，0x21：战斗机，0x22：运输预警轰炸机，0x23：直升机，0x24：其他飞机，0x31：航母，0x32：驱护舰，0x33：大型邮轮，0x34：大型货船，0x35：其他舰船，其他预留
    uint8_t targetTypeConfidence; // 目标类型置信度 UINT8 1 对应值域：0~100，单位：%，LSB：1
    uint8_t targetTypeConfidenceVec[20] = {
        0}; // 目标类型置信度向量 UINT8[20] 20 // // //
            // a[0]~a[14]依次为对应上述目标类型的置信度，其中：a[0]-0x01：车辆类目标置信度，a[1]-0x02：飞机类目标置信度，a[2]-0x03：舰船类目标置信度，a[3]-0x11：小型车（坦克）目标置信度，a[4]-0x12：中型车（军用方舱车）目标置信度，a[5]-0x13：其他车辆目标置信度，a[6]-0x21：战斗机目标置信度，a[7]-0x22：运输预警轰炸机目标置信度，a[8]-0x23：直升机目标置信度，a[9]-0x24：其他飞机目标置信度，a[10]-0x31：航母目标置信度，a[11]-0x32：驱护舰目标置信度，a[12]-0x33：大型邮轮目标置信度，a[13]-0x34：大型货船目标置信度，a[14]-0x35：其他舰船目标置信度，其他预留，对应值域：0~100，单位：%，LSB：1
    uint8_t targetWZType; // 目标WZ类型 UINT8 1 // // //
                          // 0x01：无WZ，0x02：光学WZ，0x03：全频谱WZ，0x04：舰船WZ，0x05：充气假目标，其他预留
    uint8_t targetWZTypeConfidence; // 目标WZ类型置信度 UINT8 1 对应值域：0~100，单位：%，LSB：1
    uint8_t targetWZTypeConfidenceVec[8] = {
        0}; // 目标WZ类型置信度向量 UINT8[8] 8 // // //
            // a[0]~a[4]依次为对应上述目标WZ类型的置信度，其中：a[0]-0x01：无WZ目标置信度，a[1]-0x02：光学WZ目标置信度，a[2]-0x03：全频谱WZ目标置信度，a[3]-0x04：舰船WZ目标置信度，a[4]-0x05：充气假目标目标置信度，其他预留，对应值域：0~100，单位：%，LSB：1

    TargetMsg kjg_targtet_msg; // 可见光
    TargetMsg hw_targtet_msg;  // 红外
    TargetMsg sar_targtet_msg; // SAR

    uint32_t tail = 0xAAAAAAAA; // 帧尾，固定值：0xAAAAAAAA
};

struct TargDetectionImgMsg1
{
    uint32_t frame_header = 0x55555555; // 帧头，固定值：0x55555555
    uint32_t message_length;            // 消息长度，包括从帧头到帧尾的所有字段字节长度
    uint8_t data_type;                  // 数据类型，0x01：红外，0x02：光学，0x03：SAR
    uint8_t image_name[256] = {0};      // 图像名称，字符串路径
    uint8_t image_source;      // 图像来源，0x01：卫星平台，0x02：机载平台，0x03：无人机平台
    uint8_t work_mode;         // 工作模式，0x01：条带，0x02：聚束
    uint16_t image_resolution; // 图像分辨率，单位：米，LSB：0.01
    uint32_t image_rows;       // 图像行数，单张图像的行数，沿平台航迹方向，正交于波束指向
    uint32_t image_cols;       // 图像列数，单张图像的列数，沿波束指向，图像条幅宽度
    uint8_t quantization_bits; // 量化位数，0x01：8bit；

    PositionInformationOfImageFeaturePoints imageFeaturePosition; // 目标信息，最多4个目标

    uint16_t total_package_num;         // 图像数据的总包数，取值范围：[]
    uint8_t sub_package_index[4] = {0}; // 子包序号，取值范围；1~0xFFFFFF。
    uint32_t valid_data_length;         // 有效数据长度N，取值范围：[0~256K]
};

struct TargDetectionImgMsg2
{
    uint8_t reserved[2] = {0, 0}; // 保留， 默认0
    uint32_t frame_tail;          // 帧尾，固定值：0xAAAAAAAA
};

#pragma pack()
#endif // TARGDETECTIONMSG_H