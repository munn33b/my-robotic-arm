    else if(in->topicType == "builtin_interfaces/msg/Duration")
    {
        auto cb = [=](const builtin_interfaces::msg::Duration::SharedPtr msg) { ros_callback__builtin_interfaces__msg__Duration(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<builtin_interfaces::msg::Duration>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "builtin_interfaces/msg/Time")
    {
        auto cb = [=](const builtin_interfaces::msg::Time::SharedPtr msg) { ros_callback__builtin_interfaces__msg__Time(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<builtin_interfaces::msg::Time>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "custom_message/msg/Floats")
    {
        auto cb = [=](const custom_message::msg::Floats::SharedPtr msg) { ros_callback__custom_message__msg__Floats(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<custom_message::msg::Floats>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "geometry_msgs/msg/Point")
    {
        auto cb = [=](const geometry_msgs::msg::Point::SharedPtr msg) { ros_callback__geometry_msgs__msg__Point(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::Point>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "geometry_msgs/msg/Pose")
    {
        auto cb = [=](const geometry_msgs::msg::Pose::SharedPtr msg) { ros_callback__geometry_msgs__msg__Pose(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::Pose>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "geometry_msgs/msg/PoseWithCovariance")
    {
        auto cb = [=](const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg) { ros_callback__geometry_msgs__msg__PoseWithCovariance(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::PoseWithCovariance>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "geometry_msgs/msg/Quaternion")
    {
        auto cb = [=](const geometry_msgs::msg::Quaternion::SharedPtr msg) { ros_callback__geometry_msgs__msg__Quaternion(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::Quaternion>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "geometry_msgs/msg/Transform")
    {
        auto cb = [=](const geometry_msgs::msg::Transform::SharedPtr msg) { ros_callback__geometry_msgs__msg__Transform(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::Transform>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "geometry_msgs/msg/TransformStamped")
    {
        auto cb = [=](const geometry_msgs::msg::TransformStamped::SharedPtr msg) { ros_callback__geometry_msgs__msg__TransformStamped(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::TransformStamped>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "geometry_msgs/msg/Twist")
    {
        auto cb = [=](const geometry_msgs::msg::Twist::SharedPtr msg) { ros_callback__geometry_msgs__msg__Twist(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::Twist>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "geometry_msgs/msg/TwistWithCovariance")
    {
        auto cb = [=](const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg) { ros_callback__geometry_msgs__msg__TwistWithCovariance(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::TwistWithCovariance>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "geometry_msgs/msg/Vector3")
    {
        auto cb = [=](const geometry_msgs::msg::Vector3::SharedPtr msg) { ros_callback__geometry_msgs__msg__Vector3(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::Vector3>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "nav_msgs/msg/Odometry")
    {
        auto cb = [=](const nav_msgs::msg::Odometry::SharedPtr msg) { ros_callback__nav_msgs__msg__Odometry(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<nav_msgs::msg::Odometry>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "sensor_msgs/msg/Image")
    {
        auto cb = [=](const sensor_msgs::msg::Image::SharedPtr msg) { ros_callback__sensor_msgs__msg__Image(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<sensor_msgs::msg::Image>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "sensor_msgs/msg/LaserScan")
    {
        auto cb = [=](const sensor_msgs::msg::LaserScan::SharedPtr msg) { ros_callback__sensor_msgs__msg__LaserScan(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "sensor_msgs/msg/PointCloud2")
    {
        auto cb = [=](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { ros_callback__sensor_msgs__msg__PointCloud2(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "sensor_msgs/msg/PointField")
    {
        auto cb = [=](const sensor_msgs::msg::PointField::SharedPtr msg) { ros_callback__sensor_msgs__msg__PointField(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<sensor_msgs::msg::PointField>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "sensor_msgs/msg/Range")
    {
        auto cb = [=](const sensor_msgs::msg::Range::SharedPtr msg) { ros_callback__sensor_msgs__msg__Range(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<sensor_msgs::msg::Range>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/Bool")
    {
        auto cb = [=](const std_msgs::msg::Bool::SharedPtr msg) { ros_callback__std_msgs__msg__Bool(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Bool>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/Byte")
    {
        auto cb = [=](const std_msgs::msg::Byte::SharedPtr msg) { ros_callback__std_msgs__msg__Byte(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Byte>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/ColorRGBA")
    {
        auto cb = [=](const std_msgs::msg::ColorRGBA::SharedPtr msg) { ros_callback__std_msgs__msg__ColorRGBA(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::ColorRGBA>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/Empty")
    {
        auto cb = [=](const std_msgs::msg::Empty::SharedPtr msg) { ros_callback__std_msgs__msg__Empty(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Empty>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/Float32")
    {
        auto cb = [=](const std_msgs::msg::Float32::SharedPtr msg) { ros_callback__std_msgs__msg__Float32(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Float32>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/Float64")
    {
        auto cb = [=](const std_msgs::msg::Float64::SharedPtr msg) { ros_callback__std_msgs__msg__Float64(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Float64>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/Header")
    {
        auto cb = [=](const std_msgs::msg::Header::SharedPtr msg) { ros_callback__std_msgs__msg__Header(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Header>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/Int16")
    {
        auto cb = [=](const std_msgs::msg::Int16::SharedPtr msg) { ros_callback__std_msgs__msg__Int16(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Int16>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/Int32")
    {
        auto cb = [=](const std_msgs::msg::Int32::SharedPtr msg) { ros_callback__std_msgs__msg__Int32(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Int32>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/Int64")
    {
        auto cb = [=](const std_msgs::msg::Int64::SharedPtr msg) { ros_callback__std_msgs__msg__Int64(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Int64>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/Int8")
    {
        auto cb = [=](const std_msgs::msg::Int8::SharedPtr msg) { ros_callback__std_msgs__msg__Int8(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Int8>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/String")
    {
        auto cb = [=](const std_msgs::msg::String::SharedPtr msg) { ros_callback__std_msgs__msg__String(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::String>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/UInt16")
    {
        auto cb = [=](const std_msgs::msg::UInt16::SharedPtr msg) { ros_callback__std_msgs__msg__UInt16(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::UInt16>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/UInt32")
    {
        auto cb = [=](const std_msgs::msg::UInt32::SharedPtr msg) { ros_callback__std_msgs__msg__UInt32(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::UInt32>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/UInt64")
    {
        auto cb = [=](const std_msgs::msg::UInt64::SharedPtr msg) { ros_callback__std_msgs__msg__UInt64(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::UInt64>(in->topicName, get_qos(in->qos), cb);
    }
    else if(in->topicType == "std_msgs/msg/UInt8")
    {
        auto cb = [=](const std_msgs::msg::UInt8::SharedPtr msg) { ros_callback__std_msgs__msg__UInt8(msg, subscriptionProxy); };
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::UInt8>(in->topicName, get_qos(in->qos), cb);
    }
