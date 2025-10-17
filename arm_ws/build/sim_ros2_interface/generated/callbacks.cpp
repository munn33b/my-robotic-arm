#include <callbacks.h>
#include <simLib/simLib.h>
#include <stubs.h>
#include <cstring>

void write__builtin_interfaces__msg__Duration(const builtin_interfaces::msg::Duration& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'sec'
            sim::pushStringOntoStack(stack, "sec", 0);
            write__int32(msg.sec, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'sec': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'nanosec'
            sim::pushStringOntoStack(stack, "nanosec", 0);
            write__uint32(msg.nanosec, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'nanosec': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__builtin_interfaces__msg__Duration: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__builtin_interfaces__msg__Duration(int stack, builtin_interfaces::msg::Duration *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "sec") == 0)
                {
                    try
                    {
                        // read field 'sec'
                        read__int32(stack, &(msg->sec), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field sec: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "nanosec") == 0)
                {
                    try
                    {
                        // read field 'nanosec'
                        read__uint32(stack, &(msg->nanosec), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field nanosec: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__builtin_interfaces__msg__Duration: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__builtin_interfaces__msg__Time(const builtin_interfaces::msg::Time& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'sec'
            sim::pushStringOntoStack(stack, "sec", 0);
            write__int32(msg.sec, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'sec': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'nanosec'
            sim::pushStringOntoStack(stack, "nanosec", 0);
            write__uint32(msg.nanosec, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'nanosec': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__builtin_interfaces__msg__Time: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__builtin_interfaces__msg__Time(int stack, builtin_interfaces::msg::Time *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "sec") == 0)
                {
                    try
                    {
                        // read field 'sec'
                        read__int32(stack, &(msg->sec), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field sec: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "nanosec") == 0)
                {
                    try
                    {
                        // read field 'nanosec'
                        read__uint32(stack, &(msg->nanosec), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field nanosec: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__builtin_interfaces__msg__Time: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__custom_message__msg__Floats(const custom_message::msg::Floats& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data' (using fast specialized function)
            sim::pushStringOntoStack(stack, "data", 0);
            sim::pushFloatTableOntoStack(stack, &(msg.data[0]), msg.data.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__custom_message__msg__Floats: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__custom_message__msg__Floats(int stack, custom_message::msg::Floats *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data' (using fast specialized function)
                        int sz = sim::getStackTableInfo(stack, 0);
                        if(sz < 0)
                            throw sim::exception("expected array");
                        if(sim::getStackTableInfo(stack, 2) != 1)
                            throw sim::exception("fast_write_type reader exception #1");
                        msg->data.resize(sz);
                        sim::getStackFloatTable(stack, &(msg->data[0]), sz);
                        sim::popStackItem(stack, 1);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__custom_message__msg__Floats: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__example_interfaces__action__Fibonacci__Goal(const example_interfaces::action::Fibonacci::Goal& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'order'
            sim::pushStringOntoStack(stack, "order", 0);
            write__int32(msg.order, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'order': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__example_interfaces__action__Fibonacci__Goal: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__example_interfaces__action__Fibonacci__Goal(int stack, example_interfaces::action::Fibonacci::Goal *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "order") == 0)
                {
                    try
                    {
                        // read field 'order'
                        read__int32(stack, &(msg->order), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field order: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__example_interfaces__action__Fibonacci__Goal: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__example_interfaces__action__Fibonacci__Feedback(const example_interfaces::action::Fibonacci::Feedback& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'sequence' (using fast specialized function)
            sim::pushStringOntoStack(stack, "sequence", 0);
            sim::pushInt32TableOntoStack(stack, &(msg.sequence[0]), msg.sequence.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'sequence': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__example_interfaces__action__Fibonacci__Feedback: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__example_interfaces__action__Fibonacci__Feedback(int stack, example_interfaces::action::Fibonacci::Feedback *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "sequence") == 0)
                {
                    try
                    {
                        // read field 'sequence' (using fast specialized function)
                        int sz = sim::getStackTableInfo(stack, 0);
                        if(sz < 0)
                            throw sim::exception("expected array");
                        if(sim::getStackTableInfo(stack, 2) != 1)
                            throw sim::exception("fast_write_type reader exception #1");
                        msg->sequence.resize(sz);
                        sim::getStackInt32Table(stack, &(msg->sequence[0]), sz);
                        sim::popStackItem(stack, 1);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field sequence: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__example_interfaces__action__Fibonacci__Feedback: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__example_interfaces__action__Fibonacci__Result(const example_interfaces::action::Fibonacci::Result& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'sequence' (using fast specialized function)
            sim::pushStringOntoStack(stack, "sequence", 0);
            sim::pushInt32TableOntoStack(stack, &(msg.sequence[0]), msg.sequence.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'sequence': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__example_interfaces__action__Fibonacci__Result: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__example_interfaces__action__Fibonacci__Result(int stack, example_interfaces::action::Fibonacci::Result *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "sequence") == 0)
                {
                    try
                    {
                        // read field 'sequence' (using fast specialized function)
                        int sz = sim::getStackTableInfo(stack, 0);
                        if(sz < 0)
                            throw sim::exception("expected array");
                        if(sim::getStackTableInfo(stack, 2) != 1)
                            throw sim::exception("fast_write_type reader exception #1");
                        msg->sequence.resize(sz);
                        sim::getStackInt32Table(stack, &(msg->sequence[0]), sz);
                        sim::popStackItem(stack, 1);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field sequence: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__example_interfaces__action__Fibonacci__Result: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__example_interfaces__srv__AddTwoInts__Request(const example_interfaces::srv::AddTwoInts::Request& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'a'
            sim::pushStringOntoStack(stack, "a", 0);
            write__int64(msg.a, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'a': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'b'
            sim::pushStringOntoStack(stack, "b", 0);
            write__int64(msg.b, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'b': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__example_interfaces__srv__AddTwoInts__Request: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__example_interfaces__srv__AddTwoInts__Request(int stack, example_interfaces::srv::AddTwoInts::Request *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "a") == 0)
                {
                    try
                    {
                        // read field 'a'
                        read__int64(stack, &(msg->a), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field a: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "b") == 0)
                {
                    try
                    {
                        // read field 'b'
                        read__int64(stack, &(msg->b), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field b: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__example_interfaces__srv__AddTwoInts__Request: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__example_interfaces__srv__AddTwoInts__Response(const example_interfaces::srv::AddTwoInts::Response& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'sum'
            sim::pushStringOntoStack(stack, "sum", 0);
            write__int64(msg.sum, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'sum': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__example_interfaces__srv__AddTwoInts__Response: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__example_interfaces__srv__AddTwoInts__Response(int stack, example_interfaces::srv::AddTwoInts::Response *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "sum") == 0)
                {
                    try
                    {
                        // read field 'sum'
                        read__int64(stack, &(msg->sum), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field sum: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__example_interfaces__srv__AddTwoInts__Response: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__geometry_msgs__msg__Point(const geometry_msgs::msg::Point& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'x'
            sim::pushStringOntoStack(stack, "x", 0);
            write__float64(msg.x, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'x': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'y'
            sim::pushStringOntoStack(stack, "y", 0);
            write__float64(msg.y, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'y': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'z'
            sim::pushStringOntoStack(stack, "z", 0);
            write__float64(msg.z, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'z': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__geometry_msgs__msg__Point: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__geometry_msgs__msg__Point(int stack, geometry_msgs::msg::Point *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "x") == 0)
                {
                    try
                    {
                        // read field 'x'
                        read__float64(stack, &(msg->x), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field x: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "y") == 0)
                {
                    try
                    {
                        // read field 'y'
                        read__float64(stack, &(msg->y), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field y: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "z") == 0)
                {
                    try
                    {
                        // read field 'z'
                        read__float64(stack, &(msg->z), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field z: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__geometry_msgs__msg__Point: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__geometry_msgs__msg__Pose(const geometry_msgs::msg::Pose& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'position'
            sim::pushStringOntoStack(stack, "position", 0);
            write__geometry_msgs__msg__Point(msg.position, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'position': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'orientation'
            sim::pushStringOntoStack(stack, "orientation", 0);
            write__geometry_msgs__msg__Quaternion(msg.orientation, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'orientation': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__geometry_msgs__msg__Pose: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__geometry_msgs__msg__Pose(int stack, geometry_msgs::msg::Pose *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "position") == 0)
                {
                    try
                    {
                        // read field 'position'
                        read__geometry_msgs__msg__Point(stack, &(msg->position), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field position: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "orientation") == 0)
                {
                    try
                    {
                        // read field 'orientation'
                        read__geometry_msgs__msg__Quaternion(stack, &(msg->orientation), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field orientation: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__geometry_msgs__msg__Pose: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__geometry_msgs__msg__PoseWithCovariance(const geometry_msgs::msg::PoseWithCovariance& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'pose'
            sim::pushStringOntoStack(stack, "pose", 0);
            write__geometry_msgs__msg__Pose(msg.pose, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'pose': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'covariance' (using fast specialized function)
            sim::pushStringOntoStack(stack, "covariance", 0);
            sim::pushDoubleTableOntoStack(stack, &(msg.covariance[0]), msg.covariance.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'covariance': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__geometry_msgs__msg__PoseWithCovariance: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__geometry_msgs__msg__PoseWithCovariance(int stack, geometry_msgs::msg::PoseWithCovariance *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "pose") == 0)
                {
                    try
                    {
                        // read field 'pose'
                        read__geometry_msgs__msg__Pose(stack, &(msg->pose), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field pose: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "covariance") == 0)
                {
                    try
                    {
                        // read field 'covariance' (using fast specialized function)
                        int sz = sim::getStackTableInfo(stack, 0);
                        if(sz < 0)
                            throw sim::exception("expected array");
                        if(sim::getStackTableInfo(stack, 2) != 1)
                            throw sim::exception("fast_write_type reader exception #1");
                        // field has fixed size -> no need to reserve space into vector
                        sim::getStackDoubleTable(stack, &(msg->covariance[0]), sz);
                        sim::popStackItem(stack, 1);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field covariance: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__geometry_msgs__msg__PoseWithCovariance: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__geometry_msgs__msg__Quaternion(const geometry_msgs::msg::Quaternion& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'x'
            sim::pushStringOntoStack(stack, "x", 0);
            write__float64(msg.x, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'x': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'y'
            sim::pushStringOntoStack(stack, "y", 0);
            write__float64(msg.y, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'y': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'z'
            sim::pushStringOntoStack(stack, "z", 0);
            write__float64(msg.z, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'z': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'w'
            sim::pushStringOntoStack(stack, "w", 0);
            write__float64(msg.w, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'w': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__geometry_msgs__msg__Quaternion: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__geometry_msgs__msg__Quaternion(int stack, geometry_msgs::msg::Quaternion *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "x") == 0)
                {
                    try
                    {
                        // read field 'x'
                        read__float64(stack, &(msg->x), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field x: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "y") == 0)
                {
                    try
                    {
                        // read field 'y'
                        read__float64(stack, &(msg->y), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field y: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "z") == 0)
                {
                    try
                    {
                        // read field 'z'
                        read__float64(stack, &(msg->z), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field z: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "w") == 0)
                {
                    try
                    {
                        // read field 'w'
                        read__float64(stack, &(msg->w), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field w: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__geometry_msgs__msg__Quaternion: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__geometry_msgs__msg__Transform(const geometry_msgs::msg::Transform& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'translation'
            sim::pushStringOntoStack(stack, "translation", 0);
            write__geometry_msgs__msg__Vector3(msg.translation, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'translation': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'rotation'
            sim::pushStringOntoStack(stack, "rotation", 0);
            write__geometry_msgs__msg__Quaternion(msg.rotation, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'rotation': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__geometry_msgs__msg__Transform: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__geometry_msgs__msg__Transform(int stack, geometry_msgs::msg::Transform *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "translation") == 0)
                {
                    try
                    {
                        // read field 'translation'
                        read__geometry_msgs__msg__Vector3(stack, &(msg->translation), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field translation: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "rotation") == 0)
                {
                    try
                    {
                        // read field 'rotation'
                        read__geometry_msgs__msg__Quaternion(stack, &(msg->rotation), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field rotation: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__geometry_msgs__msg__Transform: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__geometry_msgs__msg__TransformStamped(const geometry_msgs::msg::TransformStamped& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'header'
            sim::pushStringOntoStack(stack, "header", 0);
            write__std_msgs__msg__Header(msg.header, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'header': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'child_frame_id'
            sim::pushStringOntoStack(stack, "child_frame_id", 0);
            write__string(msg.child_frame_id, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'child_frame_id': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'transform'
            sim::pushStringOntoStack(stack, "transform", 0);
            write__geometry_msgs__msg__Transform(msg.transform, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'transform': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__geometry_msgs__msg__TransformStamped: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__geometry_msgs__msg__TransformStamped(int stack, geometry_msgs::msg::TransformStamped *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "header") == 0)
                {
                    try
                    {
                        // read field 'header'
                        read__std_msgs__msg__Header(stack, &(msg->header), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field header: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "child_frame_id") == 0)
                {
                    try
                    {
                        // read field 'child_frame_id'
                        read__string(stack, &(msg->child_frame_id), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field child_frame_id: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "transform") == 0)
                {
                    try
                    {
                        // read field 'transform'
                        read__geometry_msgs__msg__Transform(stack, &(msg->transform), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field transform: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__geometry_msgs__msg__TransformStamped: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__geometry_msgs__msg__Twist(const geometry_msgs::msg::Twist& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'linear'
            sim::pushStringOntoStack(stack, "linear", 0);
            write__geometry_msgs__msg__Vector3(msg.linear, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'linear': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'angular'
            sim::pushStringOntoStack(stack, "angular", 0);
            write__geometry_msgs__msg__Vector3(msg.angular, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'angular': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__geometry_msgs__msg__Twist: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__geometry_msgs__msg__Twist(int stack, geometry_msgs::msg::Twist *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "linear") == 0)
                {
                    try
                    {
                        // read field 'linear'
                        read__geometry_msgs__msg__Vector3(stack, &(msg->linear), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field linear: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "angular") == 0)
                {
                    try
                    {
                        // read field 'angular'
                        read__geometry_msgs__msg__Vector3(stack, &(msg->angular), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field angular: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__geometry_msgs__msg__Twist: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__geometry_msgs__msg__TwistWithCovariance(const geometry_msgs::msg::TwistWithCovariance& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'twist'
            sim::pushStringOntoStack(stack, "twist", 0);
            write__geometry_msgs__msg__Twist(msg.twist, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'twist': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'covariance' (using fast specialized function)
            sim::pushStringOntoStack(stack, "covariance", 0);
            sim::pushDoubleTableOntoStack(stack, &(msg.covariance[0]), msg.covariance.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'covariance': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__geometry_msgs__msg__TwistWithCovariance: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__geometry_msgs__msg__TwistWithCovariance(int stack, geometry_msgs::msg::TwistWithCovariance *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "twist") == 0)
                {
                    try
                    {
                        // read field 'twist'
                        read__geometry_msgs__msg__Twist(stack, &(msg->twist), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field twist: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "covariance") == 0)
                {
                    try
                    {
                        // read field 'covariance' (using fast specialized function)
                        int sz = sim::getStackTableInfo(stack, 0);
                        if(sz < 0)
                            throw sim::exception("expected array");
                        if(sim::getStackTableInfo(stack, 2) != 1)
                            throw sim::exception("fast_write_type reader exception #1");
                        // field has fixed size -> no need to reserve space into vector
                        sim::getStackDoubleTable(stack, &(msg->covariance[0]), sz);
                        sim::popStackItem(stack, 1);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field covariance: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__geometry_msgs__msg__TwistWithCovariance: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__geometry_msgs__msg__Vector3(const geometry_msgs::msg::Vector3& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'x'
            sim::pushStringOntoStack(stack, "x", 0);
            write__float64(msg.x, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'x': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'y'
            sim::pushStringOntoStack(stack, "y", 0);
            write__float64(msg.y, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'y': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'z'
            sim::pushStringOntoStack(stack, "z", 0);
            write__float64(msg.z, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'z': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__geometry_msgs__msg__Vector3: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__geometry_msgs__msg__Vector3(int stack, geometry_msgs::msg::Vector3 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "x") == 0)
                {
                    try
                    {
                        // read field 'x'
                        read__float64(stack, &(msg->x), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field x: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "y") == 0)
                {
                    try
                    {
                        // read field 'y'
                        read__float64(stack, &(msg->y), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field y: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "z") == 0)
                {
                    try
                    {
                        // read field 'z'
                        read__float64(stack, &(msg->z), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field z: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__geometry_msgs__msg__Vector3: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__nav_msgs__msg__Odometry(const nav_msgs::msg::Odometry& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'header'
            sim::pushStringOntoStack(stack, "header", 0);
            write__std_msgs__msg__Header(msg.header, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'header': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'child_frame_id'
            sim::pushStringOntoStack(stack, "child_frame_id", 0);
            write__string(msg.child_frame_id, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'child_frame_id': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'pose'
            sim::pushStringOntoStack(stack, "pose", 0);
            write__geometry_msgs__msg__PoseWithCovariance(msg.pose, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'pose': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'twist'
            sim::pushStringOntoStack(stack, "twist", 0);
            write__geometry_msgs__msg__TwistWithCovariance(msg.twist, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'twist': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__nav_msgs__msg__Odometry: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__nav_msgs__msg__Odometry(int stack, nav_msgs::msg::Odometry *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "header") == 0)
                {
                    try
                    {
                        // read field 'header'
                        read__std_msgs__msg__Header(stack, &(msg->header), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field header: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "child_frame_id") == 0)
                {
                    try
                    {
                        // read field 'child_frame_id'
                        read__string(stack, &(msg->child_frame_id), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field child_frame_id: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "pose") == 0)
                {
                    try
                    {
                        // read field 'pose'
                        read__geometry_msgs__msg__PoseWithCovariance(stack, &(msg->pose), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field pose: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "twist") == 0)
                {
                    try
                    {
                        // read field 'twist'
                        read__geometry_msgs__msg__TwistWithCovariance(stack, &(msg->twist), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field twist: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__nav_msgs__msg__Odometry: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__sensor_msgs__msg__Image(const sensor_msgs::msg::Image& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'header'
            sim::pushStringOntoStack(stack, "header", 0);
            write__std_msgs__msg__Header(msg.header, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'header': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'height'
            sim::pushStringOntoStack(stack, "height", 0);
            write__uint32(msg.height, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'height': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'width'
            sim::pushStringOntoStack(stack, "width", 0);
            write__uint32(msg.width, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'width': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'encoding'
            sim::pushStringOntoStack(stack, "encoding", 0);
            write__string(msg.encoding, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'encoding': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'is_bigendian'
            sim::pushStringOntoStack(stack, "is_bigendian", 0);
            write__uint8(msg.is_bigendian, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'is_bigendian': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'step'
            sim::pushStringOntoStack(stack, "step", 0);
            write__uint32(msg.step, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'step': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'data' (using fast specialized function)
            sim::pushStringOntoStack(stack, "data", 0);
            if(opt && opt->uint8array_as_string)
                sim::pushStringOntoStack(stack, (char*)&(msg.data[0]), msg.data.size());
            else
                sim::pushUInt8TableOntoStack(stack, &(msg.data[0]), msg.data.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__sensor_msgs__msg__Image: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__sensor_msgs__msg__Image(int stack, sensor_msgs::msg::Image *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "header") == 0)
                {
                    try
                    {
                        // read field 'header'
                        read__std_msgs__msg__Header(stack, &(msg->header), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field header: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "height") == 0)
                {
                    try
                    {
                        // read field 'height'
                        read__uint32(stack, &(msg->height), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field height: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "width") == 0)
                {
                    try
                    {
                        // read field 'width'
                        read__uint32(stack, &(msg->width), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field width: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "encoding") == 0)
                {
                    try
                    {
                        // read field 'encoding'
                        read__string(stack, &(msg->encoding), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field encoding: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "is_bigendian") == 0)
                {
                    try
                    {
                        // read field 'is_bigendian'
                        read__uint8(stack, &(msg->is_bigendian), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field is_bigendian: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "step") == 0)
                {
                    try
                    {
                        // read field 'step'
                        read__uint32(stack, &(msg->step), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field step: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        if(opt && opt->uint8array_as_string)
                        {
                            // read field 'data' (uint8[]) as string
                            char *str;
                            int sz;
                            if((str = sim::getStackStringValue(stack, &sz)) != NULL && sz > 0)
                            {
                                /*
                                 * XXX: if an alternative version of simGetStackStringValue woudl exist
                                 * working on an externally allocated buffer, we won't need this memcpy:
                                 */
                                msg->data.resize(sz);
                                std::memcpy(&(msg->data[0]), str, sz);
                                sim::releaseBuffer(str);
                            }
                            else throw sim::exception("string read error when trying to read uint8[]");
                        }
                        else
			{
                            // read field 'data' (using fast specialized function)
                            int sz = sim::getStackTableInfo(stack, 0);
                            if(sz < 0)
                                throw sim::exception("expected uint8 array");
                            if(sim::getStackTableInfo(stack, 2) != 1)
                                throw sim::exception("fast_write_type uint8[] reader exception #1");
                            msg->data.resize(sz);
                            sim::getStackUInt8Table(stack, &(msg->data[0]), sz);
                            sim::popStackItem(stack, 1);
			}
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__sensor_msgs__msg__Image: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__sensor_msgs__msg__LaserScan(const sensor_msgs::msg::LaserScan& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'header'
            sim::pushStringOntoStack(stack, "header", 0);
            write__std_msgs__msg__Header(msg.header, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'header': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'angle_min'
            sim::pushStringOntoStack(stack, "angle_min", 0);
            write__float32(msg.angle_min, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'angle_min': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'angle_max'
            sim::pushStringOntoStack(stack, "angle_max", 0);
            write__float32(msg.angle_max, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'angle_max': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'angle_increment'
            sim::pushStringOntoStack(stack, "angle_increment", 0);
            write__float32(msg.angle_increment, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'angle_increment': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'time_increment'
            sim::pushStringOntoStack(stack, "time_increment", 0);
            write__float32(msg.time_increment, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'time_increment': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'scan_time'
            sim::pushStringOntoStack(stack, "scan_time", 0);
            write__float32(msg.scan_time, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'scan_time': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'range_min'
            sim::pushStringOntoStack(stack, "range_min", 0);
            write__float32(msg.range_min, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'range_min': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'range_max'
            sim::pushStringOntoStack(stack, "range_max", 0);
            write__float32(msg.range_max, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'range_max': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'ranges' (using fast specialized function)
            sim::pushStringOntoStack(stack, "ranges", 0);
            sim::pushFloatTableOntoStack(stack, &(msg.ranges[0]), msg.ranges.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'ranges': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'intensities' (using fast specialized function)
            sim::pushStringOntoStack(stack, "intensities", 0);
            sim::pushFloatTableOntoStack(stack, &(msg.intensities[0]), msg.intensities.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'intensities': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__sensor_msgs__msg__LaserScan: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__sensor_msgs__msg__LaserScan(int stack, sensor_msgs::msg::LaserScan *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "header") == 0)
                {
                    try
                    {
                        // read field 'header'
                        read__std_msgs__msg__Header(stack, &(msg->header), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field header: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "angle_min") == 0)
                {
                    try
                    {
                        // read field 'angle_min'
                        read__float32(stack, &(msg->angle_min), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field angle_min: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "angle_max") == 0)
                {
                    try
                    {
                        // read field 'angle_max'
                        read__float32(stack, &(msg->angle_max), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field angle_max: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "angle_increment") == 0)
                {
                    try
                    {
                        // read field 'angle_increment'
                        read__float32(stack, &(msg->angle_increment), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field angle_increment: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "time_increment") == 0)
                {
                    try
                    {
                        // read field 'time_increment'
                        read__float32(stack, &(msg->time_increment), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field time_increment: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "scan_time") == 0)
                {
                    try
                    {
                        // read field 'scan_time'
                        read__float32(stack, &(msg->scan_time), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field scan_time: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "range_min") == 0)
                {
                    try
                    {
                        // read field 'range_min'
                        read__float32(stack, &(msg->range_min), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field range_min: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "range_max") == 0)
                {
                    try
                    {
                        // read field 'range_max'
                        read__float32(stack, &(msg->range_max), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field range_max: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "ranges") == 0)
                {
                    try
                    {
                        // read field 'ranges' (using fast specialized function)
                        int sz = sim::getStackTableInfo(stack, 0);
                        if(sz < 0)
                            throw sim::exception("expected array");
                        if(sim::getStackTableInfo(stack, 2) != 1)
                            throw sim::exception("fast_write_type reader exception #1");
                        msg->ranges.resize(sz);
                        sim::getStackFloatTable(stack, &(msg->ranges[0]), sz);
                        sim::popStackItem(stack, 1);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field ranges: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "intensities") == 0)
                {
                    try
                    {
                        // read field 'intensities' (using fast specialized function)
                        int sz = sim::getStackTableInfo(stack, 0);
                        if(sz < 0)
                            throw sim::exception("expected array");
                        if(sim::getStackTableInfo(stack, 2) != 1)
                            throw sim::exception("fast_write_type reader exception #1");
                        msg->intensities.resize(sz);
                        sim::getStackFloatTable(stack, &(msg->intensities[0]), sz);
                        sim::popStackItem(stack, 1);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field intensities: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__sensor_msgs__msg__LaserScan: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__sensor_msgs__msg__PointCloud2(const sensor_msgs::msg::PointCloud2& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'header'
            sim::pushStringOntoStack(stack, "header", 0);
            write__std_msgs__msg__Header(msg.header, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'header': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'height'
            sim::pushStringOntoStack(stack, "height", 0);
            write__uint32(msg.height, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'height': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'width'
            sim::pushStringOntoStack(stack, "width", 0);
            write__uint32(msg.width, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'width': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'fields'
            sim::pushStringOntoStack(stack, "fields", 0);
            sim::pushTableOntoStack(stack);
            for(int i = 0; i < msg.fields.size(); i++)
            {
                write__int32(i + 1, stack, opt);
                write__sensor_msgs__msg__PointField(msg.fields[i], stack, opt);
                sim::insertDataIntoStackTable(stack);
            }
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'fields': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'is_bigendian'
            sim::pushStringOntoStack(stack, "is_bigendian", 0);
            write__bool(msg.is_bigendian, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'is_bigendian': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'point_step'
            sim::pushStringOntoStack(stack, "point_step", 0);
            write__uint32(msg.point_step, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'point_step': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'row_step'
            sim::pushStringOntoStack(stack, "row_step", 0);
            write__uint32(msg.row_step, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'row_step': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'data' (using fast specialized function)
            sim::pushStringOntoStack(stack, "data", 0);
            if(opt && opt->uint8array_as_string)
                sim::pushStringOntoStack(stack, (char*)&(msg.data[0]), msg.data.size());
            else
                sim::pushUInt8TableOntoStack(stack, &(msg.data[0]), msg.data.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'is_dense'
            sim::pushStringOntoStack(stack, "is_dense", 0);
            write__bool(msg.is_dense, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'is_dense': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__sensor_msgs__msg__PointCloud2: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__sensor_msgs__msg__PointCloud2(int stack, sensor_msgs::msg::PointCloud2 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "header") == 0)
                {
                    try
                    {
                        // read field 'header'
                        read__std_msgs__msg__Header(stack, &(msg->header), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field header: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "height") == 0)
                {
                    try
                    {
                        // read field 'height'
                        read__uint32(stack, &(msg->height), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field height: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "width") == 0)
                {
                    try
                    {
                        // read field 'width'
                        read__uint32(stack, &(msg->width), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field width: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "fields") == 0)
                {
                    try
                    {
                        // read field 'fields'
                        if(sim::getStackTableInfo(stack, 0) < 0)
                            throw sim::exception("expected array");
                        int oldsz1 = sim::getStackSize(stack);
                        sim::unfoldStackTable(stack);
                        int numItems1 = (sim::getStackSize(stack) - oldsz1 + 1) / 2;
                        for(int i = 0; i < numItems1; i++)
                        {
                            sim::moveStackItemToTop(stack, oldsz1 - 1); // move key to top
                            int j;
                            read__int32(stack, &j, opt);
                            sim::moveStackItemToTop(stack, oldsz1 - 1); // move value to top
                            sensor_msgs::msg::PointField v;
                            read__sensor_msgs__msg__PointField(stack, &v, opt);
                            msg->fields.push_back(v);
                        }
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field fields: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "is_bigendian") == 0)
                {
                    try
                    {
                        // read field 'is_bigendian'
                        read__bool(stack, &(msg->is_bigendian), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field is_bigendian: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "point_step") == 0)
                {
                    try
                    {
                        // read field 'point_step'
                        read__uint32(stack, &(msg->point_step), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field point_step: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "row_step") == 0)
                {
                    try
                    {
                        // read field 'row_step'
                        read__uint32(stack, &(msg->row_step), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field row_step: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        if(opt && opt->uint8array_as_string)
                        {
                            // read field 'data' (uint8[]) as string
                            char *str;
                            int sz;
                            if((str = sim::getStackStringValue(stack, &sz)) != NULL && sz > 0)
                            {
                                /*
                                 * XXX: if an alternative version of simGetStackStringValue woudl exist
                                 * working on an externally allocated buffer, we won't need this memcpy:
                                 */
                                msg->data.resize(sz);
                                std::memcpy(&(msg->data[0]), str, sz);
                                sim::releaseBuffer(str);
                            }
                            else throw sim::exception("string read error when trying to read uint8[]");
                        }
                        else
			{
                            // read field 'data' (using fast specialized function)
                            int sz = sim::getStackTableInfo(stack, 0);
                            if(sz < 0)
                                throw sim::exception("expected uint8 array");
                            if(sim::getStackTableInfo(stack, 2) != 1)
                                throw sim::exception("fast_write_type uint8[] reader exception #1");
                            msg->data.resize(sz);
                            sim::getStackUInt8Table(stack, &(msg->data[0]), sz);
                            sim::popStackItem(stack, 1);
			}
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "is_dense") == 0)
                {
                    try
                    {
                        // read field 'is_dense'
                        read__bool(stack, &(msg->is_dense), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field is_dense: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__sensor_msgs__msg__PointCloud2: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__sensor_msgs__msg__PointField(const sensor_msgs::msg::PointField& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'name'
            sim::pushStringOntoStack(stack, "name", 0);
            write__string(msg.name, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'name': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'offset'
            sim::pushStringOntoStack(stack, "offset", 0);
            write__uint32(msg.offset, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'offset': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'datatype'
            sim::pushStringOntoStack(stack, "datatype", 0);
            write__uint8(msg.datatype, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'datatype': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'count'
            sim::pushStringOntoStack(stack, "count", 0);
            write__uint32(msg.count, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'count': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__sensor_msgs__msg__PointField: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__sensor_msgs__msg__PointField(int stack, sensor_msgs::msg::PointField *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "name") == 0)
                {
                    try
                    {
                        // read field 'name'
                        read__string(stack, &(msg->name), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field name: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "offset") == 0)
                {
                    try
                    {
                        // read field 'offset'
                        read__uint32(stack, &(msg->offset), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field offset: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "datatype") == 0)
                {
                    try
                    {
                        // read field 'datatype'
                        read__uint8(stack, &(msg->datatype), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field datatype: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "count") == 0)
                {
                    try
                    {
                        // read field 'count'
                        read__uint32(stack, &(msg->count), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field count: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__sensor_msgs__msg__PointField: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__sensor_msgs__msg__Range(const sensor_msgs::msg::Range& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'header'
            sim::pushStringOntoStack(stack, "header", 0);
            write__std_msgs__msg__Header(msg.header, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'header': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'radiation_type'
            sim::pushStringOntoStack(stack, "radiation_type", 0);
            write__uint8(msg.radiation_type, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'radiation_type': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'field_of_view'
            sim::pushStringOntoStack(stack, "field_of_view", 0);
            write__float32(msg.field_of_view, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'field_of_view': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'min_range'
            sim::pushStringOntoStack(stack, "min_range", 0);
            write__float32(msg.min_range, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'min_range': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'max_range'
            sim::pushStringOntoStack(stack, "max_range", 0);
            write__float32(msg.max_range, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'max_range': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'range'
            sim::pushStringOntoStack(stack, "range", 0);
            write__float32(msg.range, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'range': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'variance'
            sim::pushStringOntoStack(stack, "variance", 0);
            write__float32(msg.variance, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'variance': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__sensor_msgs__msg__Range: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__sensor_msgs__msg__Range(int stack, sensor_msgs::msg::Range *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "header") == 0)
                {
                    try
                    {
                        // read field 'header'
                        read__std_msgs__msg__Header(stack, &(msg->header), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field header: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "radiation_type") == 0)
                {
                    try
                    {
                        // read field 'radiation_type'
                        read__uint8(stack, &(msg->radiation_type), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field radiation_type: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "field_of_view") == 0)
                {
                    try
                    {
                        // read field 'field_of_view'
                        read__float32(stack, &(msg->field_of_view), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field field_of_view: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "min_range") == 0)
                {
                    try
                    {
                        // read field 'min_range'
                        read__float32(stack, &(msg->min_range), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field min_range: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "max_range") == 0)
                {
                    try
                    {
                        // read field 'max_range'
                        read__float32(stack, &(msg->max_range), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field max_range: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "range") == 0)
                {
                    try
                    {
                        // read field 'range'
                        read__float32(stack, &(msg->range), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field range: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "variance") == 0)
                {
                    try
                    {
                        // read field 'variance'
                        read__float32(stack, &(msg->variance), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field variance: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__sensor_msgs__msg__Range: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__Bool(const std_msgs::msg::Bool& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__bool(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__Bool: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__Bool(int stack, std_msgs::msg::Bool *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__bool(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__Bool: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__Byte(const std_msgs::msg::Byte& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__byte(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__Byte: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__Byte(int stack, std_msgs::msg::Byte *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__byte(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__Byte: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__ColorRGBA(const std_msgs::msg::ColorRGBA& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'r'
            sim::pushStringOntoStack(stack, "r", 0);
            write__float32(msg.r, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'r': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'g'
            sim::pushStringOntoStack(stack, "g", 0);
            write__float32(msg.g, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'g': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'b'
            sim::pushStringOntoStack(stack, "b", 0);
            write__float32(msg.b, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'b': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'a'
            sim::pushStringOntoStack(stack, "a", 0);
            write__float32(msg.a, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'a': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__ColorRGBA: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__ColorRGBA(int stack, std_msgs::msg::ColorRGBA *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "r") == 0)
                {
                    try
                    {
                        // read field 'r'
                        read__float32(stack, &(msg->r), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field r: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "g") == 0)
                {
                    try
                    {
                        // read field 'g'
                        read__float32(stack, &(msg->g), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field g: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "b") == 0)
                {
                    try
                    {
                        // read field 'b'
                        read__float32(stack, &(msg->b), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field b: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "a") == 0)
                {
                    try
                    {
                        // read field 'a'
                        read__float32(stack, &(msg->a), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field a: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__ColorRGBA: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__Empty(const std_msgs::msg::Empty& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__Empty: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__Empty(int stack, std_msgs::msg::Empty *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__Empty: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__Float32(const std_msgs::msg::Float32& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__float32(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__Float32: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__Float32(int stack, std_msgs::msg::Float32 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__float32(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__Float32: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__Float64(const std_msgs::msg::Float64& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__float64(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__Float64: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__Float64(int stack, std_msgs::msg::Float64 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__float64(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__Float64: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__Header(const std_msgs::msg::Header& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'stamp'
            sim::pushStringOntoStack(stack, "stamp", 0);
            write__builtin_interfaces__msg__Time(msg.stamp, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'stamp': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'frame_id'
            sim::pushStringOntoStack(stack, "frame_id", 0);
            write__string(msg.frame_id, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'frame_id': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__Header: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__Header(int stack, std_msgs::msg::Header *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "stamp") == 0)
                {
                    try
                    {
                        // read field 'stamp'
                        read__builtin_interfaces__msg__Time(stack, &(msg->stamp), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field stamp: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "frame_id") == 0)
                {
                    try
                    {
                        // read field 'frame_id'
                        read__string(stack, &(msg->frame_id), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field frame_id: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__Header: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__Int16(const std_msgs::msg::Int16& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__int16(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__Int16: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__Int16(int stack, std_msgs::msg::Int16 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__int16(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__Int16: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__Int32(const std_msgs::msg::Int32& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__int32(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__Int32: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__Int32(int stack, std_msgs::msg::Int32 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__int32(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__Int32: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__Int64(const std_msgs::msg::Int64& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__int64(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__Int64: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__Int64(int stack, std_msgs::msg::Int64 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__int64(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__Int64: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__Int8(const std_msgs::msg::Int8& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__int8(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__Int8: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__Int8(int stack, std_msgs::msg::Int8 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__int8(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__Int8: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__String(const std_msgs::msg::String& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__string(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__String: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__String(int stack, std_msgs::msg::String *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__string(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__String: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__UInt16(const std_msgs::msg::UInt16& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__uint16(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__UInt16: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__UInt16(int stack, std_msgs::msg::UInt16 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__uint16(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__UInt16: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__UInt32(const std_msgs::msg::UInt32& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__uint32(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__UInt32: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__UInt32(int stack, std_msgs::msg::UInt32 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__uint32(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__UInt32: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__UInt64(const std_msgs::msg::UInt64& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__uint64(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__UInt64: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__UInt64(int stack, std_msgs::msg::UInt64 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__uint64(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__UInt64: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_msgs__msg__UInt8(const std_msgs::msg::UInt8& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__uint8(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_msgs__msg__UInt8: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_msgs__msg__UInt8(int stack, std_msgs::msg::UInt8 *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__uint8(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_msgs__msg__UInt8: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_srvs__srv__Empty__Request(const std_srvs::srv::Empty::Request& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_srvs__srv__Empty__Request: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_srvs__srv__Empty__Request(int stack, std_srvs::srv::Empty::Request *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_srvs__srv__Empty__Request: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_srvs__srv__Empty__Response(const std_srvs::srv::Empty::Response& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_srvs__srv__Empty__Response: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_srvs__srv__Empty__Response(int stack, std_srvs::srv::Empty::Response *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_srvs__srv__Empty__Response: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_srvs__srv__SetBool__Request(const std_srvs::srv::SetBool::Request& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'data'
            sim::pushStringOntoStack(stack, "data", 0);
            write__bool(msg.data, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'data': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_srvs__srv__SetBool__Request: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_srvs__srv__SetBool__Request(int stack, std_srvs::srv::SetBool::Request *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "data") == 0)
                {
                    try
                    {
                        // read field 'data'
                        read__bool(stack, &(msg->data), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field data: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_srvs__srv__SetBool__Request: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_srvs__srv__SetBool__Response(const std_srvs::srv::SetBool::Response& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'success'
            sim::pushStringOntoStack(stack, "success", 0);
            write__bool(msg.success, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'success': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'message'
            sim::pushStringOntoStack(stack, "message", 0);
            write__string(msg.message, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'message': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_srvs__srv__SetBool__Response: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_srvs__srv__SetBool__Response(int stack, std_srvs::srv::SetBool::Response *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "success") == 0)
                {
                    try
                    {
                        // read field 'success'
                        read__bool(stack, &(msg->success), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field success: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "message") == 0)
                {
                    try
                    {
                        // read field 'message'
                        read__string(stack, &(msg->message), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field message: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_srvs__srv__SetBool__Response: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_srvs__srv__Trigger__Request(const std_srvs::srv::Trigger::Request& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_srvs__srv__Trigger__Request: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_srvs__srv__Trigger__Request(int stack, std_srvs::srv::Trigger::Request *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_srvs__srv__Trigger__Request: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void write__std_srvs__srv__Trigger__Response(const std_srvs::srv::Trigger::Response& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
        try
        {
            // write field 'success'
            sim::pushStringOntoStack(stack, "success", 0);
            write__bool(msg.success, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'success': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
        try
        {
            // write field 'message'
            sim::pushStringOntoStack(stack, "message", 0);
            write__string(msg.message, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field 'message': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__std_srvs__srv__Trigger__Response: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__std_srvs__srv__Trigger__Response(int stack, std_srvs::srv::Trigger::Response *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
                else if(strcmp(str, "success") == 0)
                {
                    try
                    {
                        // read field 'success'
                        read__bool(stack, &(msg->success), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field success: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else if(strcmp(str, "message") == 0)
                {
                    try
                    {
                        // read field 'message'
                        read__string(stack, &(msg->message), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field message: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                sim::releaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__std_srvs__srv__Trigger__Response: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}


void ros_callback__builtin_interfaces__msg__Duration(const builtin_interfaces::msg::Duration::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__builtin_interfaces__msg__Duration(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__builtin_interfaces__msg__Duration: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__builtin_interfaces__msg__Time(const builtin_interfaces::msg::Time::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__builtin_interfaces__msg__Time(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__builtin_interfaces__msg__Time: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__custom_message__msg__Floats(const custom_message::msg::Floats::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__custom_message__msg__Floats(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__custom_message__msg__Floats: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__geometry_msgs__msg__Point(const geometry_msgs::msg::Point::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__geometry_msgs__msg__Point(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__geometry_msgs__msg__Point: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__geometry_msgs__msg__Pose(const geometry_msgs::msg::Pose::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__geometry_msgs__msg__Pose(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__geometry_msgs__msg__Pose: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__geometry_msgs__msg__PoseWithCovariance(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__geometry_msgs__msg__PoseWithCovariance(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__geometry_msgs__msg__PoseWithCovariance: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__geometry_msgs__msg__Quaternion(const geometry_msgs::msg::Quaternion::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__geometry_msgs__msg__Quaternion(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__geometry_msgs__msg__Quaternion: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__geometry_msgs__msg__Transform(const geometry_msgs::msg::Transform::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__geometry_msgs__msg__Transform(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__geometry_msgs__msg__Transform: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__geometry_msgs__msg__TransformStamped(const geometry_msgs::msg::TransformStamped::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__geometry_msgs__msg__TransformStamped(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__geometry_msgs__msg__TransformStamped: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__geometry_msgs__msg__Twist(const geometry_msgs::msg::Twist::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__geometry_msgs__msg__Twist(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__geometry_msgs__msg__Twist: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__geometry_msgs__msg__TwistWithCovariance(const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__geometry_msgs__msg__TwistWithCovariance(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__geometry_msgs__msg__TwistWithCovariance: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__geometry_msgs__msg__Vector3(const geometry_msgs::msg::Vector3::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__geometry_msgs__msg__Vector3(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__geometry_msgs__msg__Vector3: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__nav_msgs__msg__Odometry(const nav_msgs::msg::Odometry::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__nav_msgs__msg__Odometry(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__nav_msgs__msg__Odometry: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__sensor_msgs__msg__Image(const sensor_msgs::msg::Image::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__sensor_msgs__msg__Image(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__sensor_msgs__msg__Image: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__sensor_msgs__msg__LaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__sensor_msgs__msg__LaserScan(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__sensor_msgs__msg__LaserScan: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__sensor_msgs__msg__PointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__sensor_msgs__msg__PointCloud2(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__sensor_msgs__msg__PointCloud2: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__sensor_msgs__msg__PointField(const sensor_msgs::msg::PointField::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__sensor_msgs__msg__PointField(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__sensor_msgs__msg__PointField: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__sensor_msgs__msg__Range(const sensor_msgs::msg::Range::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__sensor_msgs__msg__Range(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__sensor_msgs__msg__Range: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__Bool(const std_msgs::msg::Bool::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__Bool(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__Bool: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__Byte(const std_msgs::msg::Byte::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__Byte(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__Byte: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__ColorRGBA(const std_msgs::msg::ColorRGBA::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__ColorRGBA(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__ColorRGBA: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__Empty(const std_msgs::msg::Empty::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__Empty(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__Empty: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__Float32(const std_msgs::msg::Float32::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__Float32(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__Float32: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__Float64(const std_msgs::msg::Float64::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__Float64(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__Float64: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__Header(const std_msgs::msg::Header::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__Header(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__Header: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__Int16(const std_msgs::msg::Int16::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__Int16(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__Int16: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__Int32(const std_msgs::msg::Int32::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__Int32(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__Int32: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__Int64(const std_msgs::msg::Int64::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__Int64(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__Int64: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__Int8(const std_msgs::msg::Int8::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__Int8(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__Int8: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__String(const std_msgs::msg::String::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__String(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__String: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__UInt16(const std_msgs::msg::UInt16::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__UInt16(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__UInt16: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__UInt32(const std_msgs::msg::UInt32::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__UInt32(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__UInt32: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__UInt64(const std_msgs::msg::UInt64::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__UInt64(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__UInt64: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_callback__std_msgs__msg__UInt8(const std_msgs::msg::UInt8::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__std_msgs__msg__UInt8(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_callback__std_msgs__msg__UInt8: ";
        msg += proxy->topicCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

bool ros_srv_callback__example_interfaces__srv__AddTwoInts(const std::shared_ptr<rmw_request_id_t> request_header, const example_interfaces::srv::AddTwoInts::Request::SharedPtr req, example_interfaces::srv::AddTwoInts::Response::SharedPtr res, ServiceProxy *proxy)
{
    bool ret = false;
    int stack = -1;

    try
    {
        stack = sim::createStack();
        write__example_interfaces__srv__AddTwoInts__Request(*req, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->serviceCallback.scriptId, proxy->serviceCallback.name.c_str(), stack);
        read__example_interfaces__srv__AddTwoInts__Response(stack, res.get(), &(proxy->rd_opt));
        sim::releaseStack(stack);
        stack = -1;
        return true;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_srv_callback__example_interfaces__srv__AddTwoInts: ";
        msg += proxy->serviceCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
        return false;
    }
}

bool ros_srv_callback__std_srvs__srv__Empty(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res, ServiceProxy *proxy)
{
    bool ret = false;
    int stack = -1;

    try
    {
        stack = sim::createStack();
        write__std_srvs__srv__Empty__Request(*req, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->serviceCallback.scriptId, proxy->serviceCallback.name.c_str(), stack);
        read__std_srvs__srv__Empty__Response(stack, res.get(), &(proxy->rd_opt));
        sim::releaseStack(stack);
        stack = -1;
        return true;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_srv_callback__std_srvs__srv__Empty: ";
        msg += proxy->serviceCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
        return false;
    }
}

bool ros_srv_callback__std_srvs__srv__SetBool(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res, ServiceProxy *proxy)
{
    bool ret = false;
    int stack = -1;

    try
    {
        stack = sim::createStack();
        write__std_srvs__srv__SetBool__Request(*req, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->serviceCallback.scriptId, proxy->serviceCallback.name.c_str(), stack);
        read__std_srvs__srv__SetBool__Response(stack, res.get(), &(proxy->rd_opt));
        sim::releaseStack(stack);
        stack = -1;
        return true;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_srv_callback__std_srvs__srv__SetBool: ";
        msg += proxy->serviceCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
        return false;
    }
}

bool ros_srv_callback__std_srvs__srv__Trigger(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res, ServiceProxy *proxy)
{
    bool ret = false;
    int stack = -1;

    try
    {
        stack = sim::createStack();
        write__std_srvs__srv__Trigger__Request(*req, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->serviceCallback.scriptId, proxy->serviceCallback.name.c_str(), stack);
        read__std_srvs__srv__Trigger__Response(stack, res.get(), &(proxy->rd_opt));
        sim::releaseStack(stack);
        stack = -1;
        return true;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_srv_callback__std_srvs__srv__Trigger: ";
        msg += proxy->serviceCallback.name + ": ";
        msg += ex.what();
        sim::setLastError(msg);
        return false;
    }
}

void ros_action_callback__example_interfaces__action__Fibonacci__Feedback(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const example_interfaces::action::Fibonacci::Feedback *feedback, ActionClientProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__string(goalUUIDtoString(goal_id), stack, &(proxy->wr_opt));
        write__example_interfaces__action__Fibonacci__Feedback(*feedback, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(scriptID, callback, stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_action_callback__example_interfaces__action__Fibonacci__Feedback: ";
        msg += std::string(callback) + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

void ros_action_callback__example_interfaces__action__Fibonacci__Result(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, int action_result_code, const example_interfaces::action::Fibonacci::Result::SharedPtr result, ActionClientProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__string(goalUUIDtoString(goal_id), stack, &(proxy->wr_opt));
        write__int32(action_result_code, stack, &(proxy->wr_opt));
        write__example_interfaces__action__Fibonacci__Result(*result, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(scriptID, callback, stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_action_callback__example_interfaces__action__Fibonacci__Result: ";
        msg += std::string(callback) + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

rclcpp_action::GoalResponse ros_action_callback__handle_goal__example_interfaces__action__Fibonacci__Goal(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const example_interfaces::action::Fibonacci::Goal *goal, ActionServerProxy *proxy)
{
    int stack = -1;
    int ret = simros2_goal_response_reject;
    try
    {
        stack = sim::createStack();
        write__string(goalUUIDtoString(goal_id), stack, &(proxy->wr_opt));
        write__example_interfaces__action__Fibonacci__Goal(*goal, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(scriptID, callback, stack);
        read__int32(stack, &ret, &(proxy->rd_opt));
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_action_callback__handle_goal__example_interfaces__action__Fibonacci__Goal: ";
        msg += std::string(callback) + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
    switch(ret)
    {
    case simros2_goal_response_reject:
        return rclcpp_action::GoalResponse::REJECT;
    case simros2_goal_response_accept_and_execute:
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    case simros2_goal_response_accept_and_defer:
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    default:
        sim::addLog(sim_verbosity_scripterrors, "invalid goal response");
        return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse ros_action_callback__handle_cancel__example_interfaces__action__Fibonacci__Goal(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const example_interfaces::action::Fibonacci::Goal *goal, ActionServerProxy *proxy)
{
    int stack = -1;
    int ret = simros2_cancel_response_reject;
    try
    {
        stack = sim::createStack();
        write__string(goalUUIDtoString(goal_id), stack, &(proxy->wr_opt));
        write__example_interfaces__action__Fibonacci__Goal(*goal, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(scriptID, callback, stack);
        read__int32(stack, &ret, &(proxy->rd_opt));
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_action_callback__handle_cancel__example_interfaces__action__Fibonacci__Goal: ";
        msg += std::string(callback) + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
    switch(ret)
    {
    case simros2_cancel_response_reject:
        return rclcpp_action::CancelResponse::REJECT;
    case simros2_cancel_response_accept:
        return rclcpp_action::CancelResponse::ACCEPT;
    default:
        sim::addLog(sim_verbosity_scripterrors, "invalid cancel response");
        return rclcpp_action::CancelResponse::REJECT;
    }
}

void ros_action_callback__handle_accepted__example_interfaces__action__Fibonacci__Goal(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const example_interfaces::action::Fibonacci::Goal *goal, ActionServerProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__string(goalUUIDtoString(goal_id), stack, &(proxy->wr_opt));
        write__example_interfaces__action__Fibonacci__Goal(*goal, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(scriptID, callback, stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            sim::releaseStack(stack); // don't throw
        std::string msg = "ros_action_callback__handle_accepted__example_interfaces__action__Fibonacci__Goal: ";
        msg += std::string(callback) + ": ";
        msg += ex.what();
        sim::setLastError(msg);
    }
}

