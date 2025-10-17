#ifndef STUBS_H__INCLUDED
#define STUBS_H__INCLUDED

#include <simLib/simExp.h>
#include <simPlusPlus/Lib.h>
#include <simStubsGen/cpp/common.h>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>

class FuncTracer
{
    int l_;
    std::string f_;
public:
    FuncTracer(const std::string &f, int l = sim_verbosity_trace);
    ~FuncTracer();
};

#ifndef __FUNC__
#ifdef __PRETTY_FUNCTION__
#define __FUNC__ __PRETTY_FUNCTION__
#else
#define __FUNC__ __func__
#endif
#endif // __FUNC__

#define TRACE_FUNC FuncTracer __funcTracer__##__LINE__((boost::format("%s:%d:%s:") % __FILE__ % __LINE__ % __FUNC__).str())

#ifdef QT_COMPIL
#include <QThread>

extern Qt::HANDLE UI_THREAD;
extern Qt::HANDLE SIM_THREAD;

std::string threadNickname();
void uiThread();
void simThread();

#define ASSERT_THREAD(ID) \
    if(UI_THREAD == NULL) {\
        sim::addLog(sim_verbosity_debug, "warning: cannot check ASSERT_THREAD(" #ID ") because global variable UI_THREAD is not set yet.");\
    } else if(strcmp(#ID, "UI") == 0) {\
        if(QThread::currentThreadId() != UI_THREAD) {\
            sim::addLog(sim_verbosity_errors, "%s:%d %s should be called from UI thread", __FILE__, __LINE__, __FUNC__);\
            exit(1);\
        }\
    } else if(strcmp(#ID, "!UI") == 0) {\
        if(QThread::currentThreadId() == UI_THREAD) {\
            sim::addLog(sim_verbosity_errors, "%s:%d %s should NOT be called from UI thread", __FILE__, __LINE__, __FUNC__);\
            exit(1);\
        }\
    } else {\
        sim::addLog(sim_verbosity_debug, "warning: cannot check ASSERT_THREAD(" #ID "). Can check only UI and !UI.");\
    }
#endif // QT_COMPIL

struct simros2_time
{
    int sec;
    int nanosec;

    simros2_time();
    simros2_time(const int &sec_, const int &nanosec_) : sec(sec_), nanosec(nanosec_) {}
};

void readFromStack(int stack, simros2_time *value, const ReadOptions &rdopt = {});
void writeToStack(const simros2_time &value, int stack, const WriteOptions &wropt = {});
struct simros2_qos
{
    int history;
    int depth;
    int reliability;
    int durability;
    simros2_time deadline;
    simros2_time lifespan;
    int liveliness;
    simros2_time liveliness_lease_duration;
    bool avoid_ros_namespace_conventions;

    simros2_qos();
    simros2_qos(const int &history_, const int &depth_, const int &reliability_, const int &durability_, const simros2_time &deadline_, const simros2_time &lifespan_, const int &liveliness_, const simros2_time &liveliness_lease_duration_, const bool &avoid_ros_namespace_conventions_) : history(history_), depth(depth_), reliability(reliability_), durability(durability_), deadline(deadline_), lifespan(lifespan_), liveliness(liveliness_), liveliness_lease_duration(liveliness_lease_duration_), avoid_ros_namespace_conventions(avoid_ros_namespace_conventions_) {}
};

void readFromStack(int stack, simros2_qos *value, const ReadOptions &rdopt = {});
void writeToStack(const simros2_qos &value, int stack, const WriteOptions &wropt = {});

bool registerScriptStuff();

enum clock_type
{
    simros2_clock_ros = 4872,
    simros2_clock_system = 4873,
    simros2_clock_steady = 4874,
};

const char* clock_type_string(clock_type x);

enum action_result_code
{
    simros2_action_result_code_succeeded = 8564,
    simros2_action_result_code_aborted = 8565,
    simros2_action_result_code_canceled = 8566,
    simros2_action_result_code_unknown = 8567,
};

const char* action_result_code_string(action_result_code x);

enum goal_response
{
    simros2_goal_response_reject = 12856,
    simros2_goal_response_accept_and_execute = 12857,
    simros2_goal_response_accept_and_defer = 12858,
};

const char* goal_response_string(goal_response x);

enum cancel_response
{
    simros2_cancel_response_reject = 16099,
    simros2_cancel_response_accept = 16100,
};

const char* cancel_response_string(cancel_response x);

enum qos_history_policy
{
    simros2_qos_history_policy_system_default = 12300,
    simros2_qos_history_policy_keep_last = 12301,
    simros2_qos_history_policy_keep_all = 12302,
};

const char* qos_history_policy_string(qos_history_policy x);

enum qos_reliability_policy
{
    simros2_qos_reliability_policy_system_default = 12400,
    simros2_qos_reliability_policy_reliable = 12401,
    simros2_qos_reliability_policy_best_effort = 12402,
};

const char* qos_reliability_policy_string(qos_reliability_policy x);

enum qos_durability_policy
{
    simros2_qos_durability_policy_system_default = 12500,
    simros2_qos_durability_policy_transient_local = 12501,
    simros2_qos_durability_policy_volatile = 12502,
};

const char* qos_durability_policy_string(qos_durability_policy x);

enum qos_liveliness_policy
{
    simros2_qos_liveliness_policy_system_default = 12600,
    simros2_qos_liveliness_policy_automatic = 12601,
    simros2_qos_liveliness_policy_manual_by_topic = 12602,
};

const char* qos_liveliness_policy_string(qos_liveliness_policy x);

struct createSubscription_in
{
    SScriptCallBack _;
    std::string topicName;
    std::string topicType;
    std::string topicCallback;
    int unused;
    std::optional< simros2_qos > qos;

    createSubscription_in();
};

struct createSubscription_out
{
    std::string subscriptionHandle;

    createSubscription_out();
};

void createSubscription_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_createSubscription(createSubscription_in *in, createSubscription_out *out);

struct shutdownSubscription_in
{
    SScriptCallBack _;
    std::string subscriptionHandle;

    shutdownSubscription_in();
};

struct shutdownSubscription_out
{

    shutdownSubscription_out();
};

void shutdownSubscription_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_shutdownSubscription(shutdownSubscription_in *in, shutdownSubscription_out *out);

struct subscriptionTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string subscriptionHandle;

    subscriptionTreatUInt8ArrayAsString_in();
};

struct subscriptionTreatUInt8ArrayAsString_out
{

    subscriptionTreatUInt8ArrayAsString_out();
};

void subscriptionTreatUInt8ArrayAsString_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_subscriptionTreatUInt8ArrayAsString(subscriptionTreatUInt8ArrayAsString_in *in, subscriptionTreatUInt8ArrayAsString_out *out);

struct createPublisher_in
{
    SScriptCallBack _;
    std::string topicName;
    std::string topicType;
    int unused;
    bool unused2;
    std::optional< simros2_qos > qos;

    createPublisher_in();
};

struct createPublisher_out
{
    std::string publisherHandle;

    createPublisher_out();
};

void createPublisher_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_createPublisher(createPublisher_in *in, createPublisher_out *out);

struct shutdownPublisher_in
{
    SScriptCallBack _;
    std::string publisherHandle;

    shutdownPublisher_in();
};

struct shutdownPublisher_out
{

    shutdownPublisher_out();
};

void shutdownPublisher_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_shutdownPublisher(shutdownPublisher_in *in, shutdownPublisher_out *out);

struct publisherTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string publisherHandle;

    publisherTreatUInt8ArrayAsString_in();
};

struct publisherTreatUInt8ArrayAsString_out
{

    publisherTreatUInt8ArrayAsString_out();
};

void publisherTreatUInt8ArrayAsString_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_publisherTreatUInt8ArrayAsString(publisherTreatUInt8ArrayAsString_in *in, publisherTreatUInt8ArrayAsString_out *out);

struct publish_in
{
    SScriptCallBack _;
    std::string publisherHandle;

    publish_in();
};

struct publish_out
{

    publish_out();
};

void publish_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_publish(publish_in *in, publish_out *out);

struct createClient_in
{
    SScriptCallBack _;
    std::string serviceName;
    std::string serviceType;

    createClient_in();
};

struct createClient_out
{
    std::string clientHandle;

    createClient_out();
};

void createClient_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_createClient(createClient_in *in, createClient_out *out);

struct shutdownClient_in
{
    SScriptCallBack _;
    std::string clientHandle;

    shutdownClient_in();
};

struct shutdownClient_out
{

    shutdownClient_out();
};

void shutdownClient_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_shutdownClient(shutdownClient_in *in, shutdownClient_out *out);

struct clientTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string clientHandle;

    clientTreatUInt8ArrayAsString_in();
};

struct clientTreatUInt8ArrayAsString_out
{

    clientTreatUInt8ArrayAsString_out();
};

void clientTreatUInt8ArrayAsString_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_clientTreatUInt8ArrayAsString(clientTreatUInt8ArrayAsString_in *in, clientTreatUInt8ArrayAsString_out *out);

struct waitForService_in
{
    SScriptCallBack _;
    std::string clientHandle;
    float timeout;

    waitForService_in();
};

struct waitForService_out
{
    bool result;

    waitForService_out();
};

void waitForService_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_waitForService(waitForService_in *in, waitForService_out *out);

struct call_in
{
    SScriptCallBack _;
    std::string clientHandle;

    call_in();
};

struct call_out
{

    call_out();
};

void call_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_call(call_in *in, call_out *out);

struct createService_in
{
    SScriptCallBack _;
    std::string serviceName;
    std::string serviceType;
    std::string serviceCallback;

    createService_in();
};

struct createService_out
{
    std::string serviceHandle;

    createService_out();
};

void createService_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_createService(createService_in *in, createService_out *out);

struct shutdownService_in
{
    SScriptCallBack _;
    std::string serviceHandle;

    shutdownService_in();
};

struct shutdownService_out
{

    shutdownService_out();
};

void shutdownService_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_shutdownService(shutdownService_in *in, shutdownService_out *out);

struct serviceTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string serviceHandle;

    serviceTreatUInt8ArrayAsString_in();
};

struct serviceTreatUInt8ArrayAsString_out
{

    serviceTreatUInt8ArrayAsString_out();
};

void serviceTreatUInt8ArrayAsString_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_serviceTreatUInt8ArrayAsString(serviceTreatUInt8ArrayAsString_in *in, serviceTreatUInt8ArrayAsString_out *out);

struct createActionClient_in
{
    SScriptCallBack _;
    std::string actionName;
    std::string actionType;
    std::string goalResponseCallback;
    std::string feedbackCallback;
    std::string resultCallback;

    createActionClient_in();
};

struct createActionClient_out
{
    std::string actionClientHandle;

    createActionClient_out();
};

void createActionClient_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_createActionClient(createActionClient_in *in, createActionClient_out *out);

struct shutdownActionClient_in
{
    SScriptCallBack _;
    std::string actionClientHandle;

    shutdownActionClient_in();
};

struct shutdownActionClient_out
{

    shutdownActionClient_out();
};

void shutdownActionClient_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_shutdownActionClient(shutdownActionClient_in *in, shutdownActionClient_out *out);

struct actionClientTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string actionClientHandle;

    actionClientTreatUInt8ArrayAsString_in();
};

struct actionClientTreatUInt8ArrayAsString_out
{

    actionClientTreatUInt8ArrayAsString_out();
};

void actionClientTreatUInt8ArrayAsString_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_actionClientTreatUInt8ArrayAsString(actionClientTreatUInt8ArrayAsString_in *in, actionClientTreatUInt8ArrayAsString_out *out);

struct spinSome_in
{
    SScriptCallBack _;

    spinSome_in();
};

struct spinSome_out
{

    spinSome_out();
};

void spinSome_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_spinSome(spinSome_in *in, spinSome_out *out);

struct sendGoal_in
{
    SScriptCallBack _;
    std::string actionClientHandle;

    sendGoal_in();
};

struct sendGoal_out
{
    bool success;

    sendGoal_out();
};

void sendGoal_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_sendGoal(sendGoal_in *in, sendGoal_out *out);

struct cancelLastGoal_in
{
    SScriptCallBack _;
    std::string actionClientHandle;

    cancelLastGoal_in();
};

struct cancelLastGoal_out
{
    bool success;

    cancelLastGoal_out();
};

void cancelLastGoal_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_cancelLastGoal(cancelLastGoal_in *in, cancelLastGoal_out *out);

struct createActionServer_in
{
    SScriptCallBack _;
    std::string actionName;
    std::string actionType;
    std::string handleGoalCallback;
    std::string handleCancelCallback;
    std::string handleAcceptedCallback;

    createActionServer_in();
};

struct createActionServer_out
{
    std::string actionServerHandle;

    createActionServer_out();
};

void createActionServer_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_createActionServer(createActionServer_in *in, createActionServer_out *out);

struct shutdownActionServer_in
{
    SScriptCallBack _;
    std::string actionServerHandle;

    shutdownActionServer_in();
};

struct shutdownActionServer_out
{

    shutdownActionServer_out();
};

void shutdownActionServer_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_shutdownActionServer(shutdownActionServer_in *in, shutdownActionServer_out *out);

struct actionServerTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string actionServerHandle;

    actionServerTreatUInt8ArrayAsString_in();
};

struct actionServerTreatUInt8ArrayAsString_out
{

    actionServerTreatUInt8ArrayAsString_out();
};

void actionServerTreatUInt8ArrayAsString_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_actionServerTreatUInt8ArrayAsString(actionServerTreatUInt8ArrayAsString_in *in, actionServerTreatUInt8ArrayAsString_out *out);

struct actionServerPublishFeedback_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerPublishFeedback_in();
};

struct actionServerPublishFeedback_out
{

    actionServerPublishFeedback_out();
};

void actionServerPublishFeedback_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_actionServerPublishFeedback(actionServerPublishFeedback_in *in, actionServerPublishFeedback_out *out);

struct actionServerActionAbort_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionAbort_in();
};

struct actionServerActionAbort_out
{

    actionServerActionAbort_out();
};

void actionServerActionAbort_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_actionServerActionAbort(actionServerActionAbort_in *in, actionServerActionAbort_out *out);

struct actionServerActionSucceed_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionSucceed_in();
};

struct actionServerActionSucceed_out
{

    actionServerActionSucceed_out();
};

void actionServerActionSucceed_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_actionServerActionSucceed(actionServerActionSucceed_in *in, actionServerActionSucceed_out *out);

struct actionServerActionCanceled_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionCanceled_in();
};

struct actionServerActionCanceled_out
{

    actionServerActionCanceled_out();
};

void actionServerActionCanceled_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_actionServerActionCanceled(actionServerActionCanceled_in *in, actionServerActionCanceled_out *out);

struct actionServerActionExecute_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionExecute_in();
};

struct actionServerActionExecute_out
{

    actionServerActionExecute_out();
};

void actionServerActionExecute_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_actionServerActionExecute(actionServerActionExecute_in *in, actionServerActionExecute_out *out);

struct actionServerActionIsCanceling_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionIsCanceling_in();
};

struct actionServerActionIsCanceling_out
{
    bool result;

    actionServerActionIsCanceling_out();
};

void actionServerActionIsCanceling_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_actionServerActionIsCanceling(actionServerActionIsCanceling_in *in, actionServerActionIsCanceling_out *out);

struct actionServerActionIsActive_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionIsActive_in();
};

struct actionServerActionIsActive_out
{
    bool result;

    actionServerActionIsActive_out();
};

void actionServerActionIsActive_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_actionServerActionIsActive(actionServerActionIsActive_in *in, actionServerActionIsActive_out *out);

struct actionServerActionIsExecuting_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionIsExecuting_in();
};

struct actionServerActionIsExecuting_out
{
    bool result;

    actionServerActionIsExecuting_out();
};

void actionServerActionIsExecuting_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_actionServerActionIsExecuting(actionServerActionIsExecuting_in *in, actionServerActionIsExecuting_out *out);

struct sendTransform_in
{
    SScriptCallBack _;

    sendTransform_in();
};

struct sendTransform_out
{

    sendTransform_out();
};

void sendTransform_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_sendTransform(sendTransform_in *in, sendTransform_out *out);

struct sendTransforms_in
{
    SScriptCallBack _;

    sendTransforms_in();
};

struct sendTransforms_out
{

    sendTransforms_out();
};

void sendTransforms_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_sendTransforms(sendTransforms_in *in, sendTransforms_out *out);

struct imageTransportCreateSubscription_in
{
    SScriptCallBack _;
    std::string topicName;
    std::string topicCallback;
    int queueSize;

    imageTransportCreateSubscription_in();
};

struct imageTransportCreateSubscription_out
{
    std::string subscriptionHandle;

    imageTransportCreateSubscription_out();
};

void imageTransportCreateSubscription_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_imageTransportCreateSubscription(imageTransportCreateSubscription_in *in, imageTransportCreateSubscription_out *out);

struct imageTransportShutdownSubscription_in
{
    SScriptCallBack _;
    std::string subscriptionHandle;

    imageTransportShutdownSubscription_in();
};

struct imageTransportShutdownSubscription_out
{

    imageTransportShutdownSubscription_out();
};

void imageTransportShutdownSubscription_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_imageTransportShutdownSubscription(imageTransportShutdownSubscription_in *in, imageTransportShutdownSubscription_out *out);

struct imageTransportCreatePublisher_in
{
    SScriptCallBack _;
    std::string topicName;
    int queueSize;

    imageTransportCreatePublisher_in();
};

struct imageTransportCreatePublisher_out
{
    std::string publisherHandle;

    imageTransportCreatePublisher_out();
};

void imageTransportCreatePublisher_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_imageTransportCreatePublisher(imageTransportCreatePublisher_in *in, imageTransportCreatePublisher_out *out);

struct imageTransportShutdownPublisher_in
{
    SScriptCallBack _;
    std::string publisherHandle;

    imageTransportShutdownPublisher_in();
};

struct imageTransportShutdownPublisher_out
{

    imageTransportShutdownPublisher_out();
};

void imageTransportShutdownPublisher_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_imageTransportShutdownPublisher(imageTransportShutdownPublisher_in *in, imageTransportShutdownPublisher_out *out);

struct imageTransportPublish_in
{
    SScriptCallBack _;
    std::string publisherHandle;
    std::string data;
    int width;
    int height;
    std::string frame_id;

    imageTransportPublish_in();
};

struct imageTransportPublish_out
{

    imageTransportPublish_out();
};

void imageTransportPublish_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_imageTransportPublish(imageTransportPublish_in *in, imageTransportPublish_out *out);

struct getTime_in
{
    SScriptCallBack _;
    int clock_type;

    getTime_in();
};

struct getTime_out
{
    simros2_time time;

    getTime_out();
};

void getTime_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_getTime(getTime_in *in, getTime_out *out);

struct getParamString_in
{
    SScriptCallBack _;
    std::string name;
    std::string defaultValue;

    getParamString_in();
};

struct getParamString_out
{
    bool exists;
    std::string value;

    getParamString_out();
};

void getParamString_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_getParamString(getParamString_in *in, getParamString_out *out);

struct getParamInt_in
{
    SScriptCallBack _;
    std::string name;
    int defaultValue;

    getParamInt_in();
};

struct getParamInt_out
{
    bool exists;
    int value;

    getParamInt_out();
};

void getParamInt_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_getParamInt(getParamInt_in *in, getParamInt_out *out);

struct getParamDouble_in
{
    SScriptCallBack _;
    std::string name;
    double defaultValue;

    getParamDouble_in();
};

struct getParamDouble_out
{
    bool exists;
    double value;

    getParamDouble_out();
};

void getParamDouble_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_getParamDouble(getParamDouble_in *in, getParamDouble_out *out);

struct getParamBool_in
{
    SScriptCallBack _;
    std::string name;
    bool defaultValue;

    getParamBool_in();
};

struct getParamBool_out
{
    bool exists;
    bool value;

    getParamBool_out();
};

void getParamBool_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_getParamBool(getParamBool_in *in, getParamBool_out *out);

struct setParamString_in
{
    SScriptCallBack _;
    std::string name;
    std::string value;

    setParamString_in();
};

struct setParamString_out
{

    setParamString_out();
};

void setParamString_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_setParamString(setParamString_in *in, setParamString_out *out);

struct setParamInt_in
{
    SScriptCallBack _;
    std::string name;
    int value;

    setParamInt_in();
};

struct setParamInt_out
{

    setParamInt_out();
};

void setParamInt_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_setParamInt(setParamInt_in *in, setParamInt_out *out);

struct setParamDouble_in
{
    SScriptCallBack _;
    std::string name;
    double value;

    setParamDouble_in();
};

struct setParamDouble_out
{

    setParamDouble_out();
};

void setParamDouble_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_setParamDouble(setParamDouble_in *in, setParamDouble_out *out);

struct setParamBool_in
{
    SScriptCallBack _;
    std::string name;
    bool value;

    setParamBool_in();
};

struct setParamBool_out
{

    setParamBool_out();
};

void setParamBool_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_setParamBool(setParamBool_in *in, setParamBool_out *out);

struct hasParam_in
{
    SScriptCallBack _;
    std::string name;

    hasParam_in();
};

struct hasParam_out
{
    bool exists;

    hasParam_out();
};

void hasParam_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_hasParam(hasParam_in *in, hasParam_out *out);

struct deleteParam_in
{
    SScriptCallBack _;
    std::string name;

    deleteParam_in();
};

struct deleteParam_out
{

    deleteParam_out();
};

void deleteParam_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_deleteParam(deleteParam_in *in, deleteParam_out *out);

struct createInterface_in
{
    SScriptCallBack _;
    std::string type;

    createInterface_in();
};

struct createInterface_out
{

    createInterface_out();
};

void createInterface_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_createInterface(createInterface_in *in, createInterface_out *out);

struct getInterfaceConstants_in
{
    SScriptCallBack _;
    std::string type;

    getInterfaceConstants_in();
};

struct getInterfaceConstants_out
{

    getInterfaceConstants_out();
};

void getInterfaceConstants_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_getInterfaceConstants(getInterfaceConstants_in *in, getInterfaceConstants_out *out);

struct supportedInterfaces_in
{
    SScriptCallBack _;

    supportedInterfaces_in();
};

struct supportedInterfaces_out
{
    std::vector< std::string > result;

    supportedInterfaces_out();
};

void supportedInterfaces_callback(SScriptCallBack *p);
SIM_DLLEXPORT void simROS2_supportedInterfaces(supportedInterfaces_in *in, supportedInterfaces_out *out);

struct subscriptionCallback_in
{

    subscriptionCallback_in();
};

struct subscriptionCallback_out
{

    subscriptionCallback_out();
};

void subscriptionCallback(int scriptId, const char *func);
bool subscriptionCallback(int scriptId, const char *func, subscriptionCallback_in *in_args, subscriptionCallback_out *out_args);

struct imageTransportCallback_in
{
    std::string data;
    int width;
    int height;

    imageTransportCallback_in();
};

struct imageTransportCallback_out
{

    imageTransportCallback_out();
};

void imageTransportCallback(int scriptId, const char *func, std::string data, int width, int height);
bool imageTransportCallback(int scriptId, const char *func, imageTransportCallback_in *in_args, imageTransportCallback_out *out_args);

struct actionGoalResponseCallback_in
{
    std::string goalID;
    bool accepted;

    actionGoalResponseCallback_in();
};

struct actionGoalResponseCallback_out
{

    actionGoalResponseCallback_out();
};

void actionGoalResponseCallback(int scriptId, const char *func, std::string goalID, bool accepted);
bool actionGoalResponseCallback(int scriptId, const char *func, actionGoalResponseCallback_in *in_args, actionGoalResponseCallback_out *out_args);

// following functions must be implemented in the plugin

void createSubscription(SScriptCallBack *p, const char *cmd, createSubscription_in *in, createSubscription_out *out);
void shutdownSubscription(SScriptCallBack *p, const char *cmd, shutdownSubscription_in *in, shutdownSubscription_out *out);
void subscriptionTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, subscriptionTreatUInt8ArrayAsString_in *in, subscriptionTreatUInt8ArrayAsString_out *out);
void createPublisher(SScriptCallBack *p, const char *cmd, createPublisher_in *in, createPublisher_out *out);
void shutdownPublisher(SScriptCallBack *p, const char *cmd, shutdownPublisher_in *in, shutdownPublisher_out *out);
void publisherTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, publisherTreatUInt8ArrayAsString_in *in, publisherTreatUInt8ArrayAsString_out *out);
void publish(SScriptCallBack *p, const char *cmd, publish_in *in, publish_out *out);
void createClient(SScriptCallBack *p, const char *cmd, createClient_in *in, createClient_out *out);
void shutdownClient(SScriptCallBack *p, const char *cmd, shutdownClient_in *in, shutdownClient_out *out);
void clientTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, clientTreatUInt8ArrayAsString_in *in, clientTreatUInt8ArrayAsString_out *out);
void waitForService(SScriptCallBack *p, const char *cmd, waitForService_in *in, waitForService_out *out);
void call(SScriptCallBack *p, const char *cmd, call_in *in, call_out *out);
void createService(SScriptCallBack *p, const char *cmd, createService_in *in, createService_out *out);
void shutdownService(SScriptCallBack *p, const char *cmd, shutdownService_in *in, shutdownService_out *out);
void serviceTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, serviceTreatUInt8ArrayAsString_in *in, serviceTreatUInt8ArrayAsString_out *out);
void createActionClient(SScriptCallBack *p, const char *cmd, createActionClient_in *in, createActionClient_out *out);
void shutdownActionClient(SScriptCallBack *p, const char *cmd, shutdownActionClient_in *in, shutdownActionClient_out *out);
void actionClientTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, actionClientTreatUInt8ArrayAsString_in *in, actionClientTreatUInt8ArrayAsString_out *out);
void spinSome(SScriptCallBack *p, const char *cmd, spinSome_in *in, spinSome_out *out);
void sendGoal(SScriptCallBack *p, const char *cmd, sendGoal_in *in, sendGoal_out *out);
void cancelLastGoal(SScriptCallBack *p, const char *cmd, cancelLastGoal_in *in, cancelLastGoal_out *out);
void createActionServer(SScriptCallBack *p, const char *cmd, createActionServer_in *in, createActionServer_out *out);
void shutdownActionServer(SScriptCallBack *p, const char *cmd, shutdownActionServer_in *in, shutdownActionServer_out *out);
void actionServerTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, actionServerTreatUInt8ArrayAsString_in *in, actionServerTreatUInt8ArrayAsString_out *out);
void actionServerPublishFeedback(SScriptCallBack *p, const char *cmd, actionServerPublishFeedback_in *in, actionServerPublishFeedback_out *out);
void actionServerActionAbort(SScriptCallBack *p, const char *cmd, actionServerActionAbort_in *in, actionServerActionAbort_out *out);
void actionServerActionSucceed(SScriptCallBack *p, const char *cmd, actionServerActionSucceed_in *in, actionServerActionSucceed_out *out);
void actionServerActionCanceled(SScriptCallBack *p, const char *cmd, actionServerActionCanceled_in *in, actionServerActionCanceled_out *out);
void actionServerActionExecute(SScriptCallBack *p, const char *cmd, actionServerActionExecute_in *in, actionServerActionExecute_out *out);
void actionServerActionIsCanceling(SScriptCallBack *p, const char *cmd, actionServerActionIsCanceling_in *in, actionServerActionIsCanceling_out *out);
void actionServerActionIsActive(SScriptCallBack *p, const char *cmd, actionServerActionIsActive_in *in, actionServerActionIsActive_out *out);
void actionServerActionIsExecuting(SScriptCallBack *p, const char *cmd, actionServerActionIsExecuting_in *in, actionServerActionIsExecuting_out *out);
void sendTransform(SScriptCallBack *p, const char *cmd, sendTransform_in *in, sendTransform_out *out);
void sendTransforms(SScriptCallBack *p, const char *cmd, sendTransforms_in *in, sendTransforms_out *out);
void imageTransportCreateSubscription(SScriptCallBack *p, const char *cmd, imageTransportCreateSubscription_in *in, imageTransportCreateSubscription_out *out);
void imageTransportShutdownSubscription(SScriptCallBack *p, const char *cmd, imageTransportShutdownSubscription_in *in, imageTransportShutdownSubscription_out *out);
void imageTransportCreatePublisher(SScriptCallBack *p, const char *cmd, imageTransportCreatePublisher_in *in, imageTransportCreatePublisher_out *out);
void imageTransportShutdownPublisher(SScriptCallBack *p, const char *cmd, imageTransportShutdownPublisher_in *in, imageTransportShutdownPublisher_out *out);
void imageTransportPublish(SScriptCallBack *p, const char *cmd, imageTransportPublish_in *in, imageTransportPublish_out *out);
void getTime(SScriptCallBack *p, const char *cmd, getTime_in *in, getTime_out *out);
void getParamString(SScriptCallBack *p, const char *cmd, getParamString_in *in, getParamString_out *out);
void getParamInt(SScriptCallBack *p, const char *cmd, getParamInt_in *in, getParamInt_out *out);
void getParamDouble(SScriptCallBack *p, const char *cmd, getParamDouble_in *in, getParamDouble_out *out);
void getParamBool(SScriptCallBack *p, const char *cmd, getParamBool_in *in, getParamBool_out *out);
void setParamString(SScriptCallBack *p, const char *cmd, setParamString_in *in, setParamString_out *out);
void setParamInt(SScriptCallBack *p, const char *cmd, setParamInt_in *in, setParamInt_out *out);
void setParamDouble(SScriptCallBack *p, const char *cmd, setParamDouble_in *in, setParamDouble_out *out);
void setParamBool(SScriptCallBack *p, const char *cmd, setParamBool_in *in, setParamBool_out *out);
void hasParam(SScriptCallBack *p, const char *cmd, hasParam_in *in, hasParam_out *out);
void deleteParam(SScriptCallBack *p, const char *cmd, deleteParam_in *in, deleteParam_out *out);
void createInterface(SScriptCallBack *p, const char *cmd, createInterface_in *in, createInterface_out *out);
void getInterfaceConstants(SScriptCallBack *p, const char *cmd, getInterfaceConstants_in *in, getInterfaceConstants_out *out);
void supportedInterfaces(SScriptCallBack *p, const char *cmd, supportedInterfaces_in *in, supportedInterfaces_out *out);

#endif // STUBS_H__INCLUDED
