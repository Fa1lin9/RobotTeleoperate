#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <iostream>
#include <thread>
#include <chrono>

// 包含你生成的消息头
#include "String_.hpp"
#include "String_PubSubTypes.hpp"

#include "Demo.hpp"
#include "DemoPubSubTypes.hpp"

#include <Ros2Bridge/Ros2Bridge.h>

using namespace eprosima::fastdds::dds;

int main()
{
//    // 1. 创建 DomainParticipant
//    DomainParticipant* participant =
//        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
//    if (!participant)
//    {
//        std::cerr << "创建 DomainParticipant 失败！" << std::endl;
//        return 1;
//    }
//    std::cout << "DomainParticipant 创建成功" << std::endl;

//    // 2. 注册类型
////    TypeSupport type(new std_msgs::msg::String_PubSubType());
//    TypeSupport type(new my_msgs::msg::DemoPubSubType());
//    if (type.register_type(participant) != RETCODE_OK)
//    {
//        std::cerr << "注册类型失败！" << std::endl;
//        return 1;
//    }

//    // 3. 创建 Topic（使用 TypeSupport 的 type name）
//    Topic* topic = participant->create_topic(
//        "rt/StringTopic",
//        type.get_type_name(),
//        TOPIC_QOS_DEFAULT
//    );
//    if (!topic)
//    {
//        std::cerr << "创建 Topic 失败！" << std::endl;
//        return 1;
//    }
//    std::cout << "Topic 创建成功: " << topic->get_name() << std::endl;

//    // 4. 创建 Publisher
//    Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
//    if (!publisher)
//    {
//        std::cerr << "创建 Publisher 失败！" << std::endl;
//        return 1;
//    }

//    // 5. 创建 DataWriter
//    DataWriterQos writer_qos = DATAWRITER_QOS_DEFAULT;
//    DataWriter* writer = publisher->create_datawriter(topic, writer_qos, nullptr);
//    if (!writer)
//    {
//        std::cerr << "创建 DataWriter 失败！" << std::endl;
//        return 1;
//    }

//    // 6. 循环发送消息
////    std_msgs::msg::String_ msg;
//    my_msgs::msg::Demo msg;
//    int count = 0;

//    while (true)
//    {
//        msg.name() = "Hello FastDDS! Count = " + std::to_string(count++);
//        if (writer->write(&msg) == RETCODE_OK)
//        {
//            std::cout << "[Publisher] Sent: " << msg.name() << std::endl;
//        }
//        else
//        {
//            std::cerr << "发送失败！" << std::endl;
//        }

//        std::this_thread::sleep_for(std::chrono::seconds(1));
//    }

//    // 7. 清理资源（示例中循环不会执行到这里）
//    participant->delete_contained_entities();
//    DomainParticipantFactory::get_instance()->delete_participant(participant);

//    return 0;
    Ros2Bridge::BasicConfig config;
    config.msgType = Ros2Bridge::MsgType::String_;
    config.topicName = "Temp";
    Ros2Bridge ros2Bridge(config);

    std_msgs::msg::String_ msg;
    int count = 0;

    while (true)
    {
        msg.data() = "Hello FastDDS! Count = " + std::to_string(count++);
        ros2Bridge.SendMsg(msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
