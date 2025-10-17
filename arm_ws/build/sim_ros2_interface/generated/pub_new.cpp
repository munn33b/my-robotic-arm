    else if(in->topicType == "builtin_interfaces/msg/Duration")
    {
        publisherProxy->publisher = node->create_publisher<builtin_interfaces::msg::Duration>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "builtin_interfaces/msg/Time")
    {
        publisherProxy->publisher = node->create_publisher<builtin_interfaces::msg::Time>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "custom_message/msg/Floats")
    {
        publisherProxy->publisher = node->create_publisher<custom_message::msg::Floats>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "geometry_msgs/msg/Point")
    {
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::Point>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "geometry_msgs/msg/Pose")
    {
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::Pose>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "geometry_msgs/msg/PoseWithCovariance")
    {
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::PoseWithCovariance>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "geometry_msgs/msg/Quaternion")
    {
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::Quaternion>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "geometry_msgs/msg/Transform")
    {
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::Transform>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "geometry_msgs/msg/TransformStamped")
    {
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::TransformStamped>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "geometry_msgs/msg/Twist")
    {
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::Twist>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "geometry_msgs/msg/TwistWithCovariance")
    {
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::TwistWithCovariance>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "geometry_msgs/msg/Vector3")
    {
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::Vector3>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "nav_msgs/msg/Odometry")
    {
        publisherProxy->publisher = node->create_publisher<nav_msgs::msg::Odometry>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "sensor_msgs/msg/Image")
    {
        publisherProxy->publisher = node->create_publisher<sensor_msgs::msg::Image>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "sensor_msgs/msg/LaserScan")
    {
        publisherProxy->publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "sensor_msgs/msg/PointCloud2")
    {
        publisherProxy->publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "sensor_msgs/msg/PointField")
    {
        publisherProxy->publisher = node->create_publisher<sensor_msgs::msg::PointField>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "sensor_msgs/msg/Range")
    {
        publisherProxy->publisher = node->create_publisher<sensor_msgs::msg::Range>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/Bool")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Bool>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/Byte")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Byte>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/ColorRGBA")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::ColorRGBA>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/Empty")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Empty>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/Float32")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Float32>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/Float64")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Float64>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/Header")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Header>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/Int16")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Int16>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/Int32")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Int32>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/Int64")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Int64>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/Int8")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Int8>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/String")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::String>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/UInt16")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::UInt16>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/UInt32")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::UInt32>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/UInt64")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::UInt64>(in->topicName, get_qos(in->qos));
    }
    else if(in->topicType == "std_msgs/msg/UInt8")
    {
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::UInt8>(in->topicName, get_qos(in->qos));
    }
