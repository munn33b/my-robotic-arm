local codeEditorInfos=[[
string subscriptionHandle = simROS2.createSubscription(string topicName, string topicType, func topicCallback, int unused=0, map qos=nil)
simROS2.shutdownSubscription(string subscriptionHandle)
simROS2.subscriptionTreatUInt8ArrayAsString(string subscriptionHandle)
string publisherHandle = simROS2.createPublisher(string topicName, string topicType, int unused=0, bool unused2=false, map qos=nil)
simROS2.shutdownPublisher(string publisherHandle)
simROS2.publisherTreatUInt8ArrayAsString(string publisherHandle)
simROS2.publish(string publisherHandle, map message)
string clientHandle = simROS2.createClient(string serviceName, string serviceType)
simROS2.shutdownClient(string clientHandle)
simROS2.clientTreatUInt8ArrayAsString(string clientHandle)
bool result = simROS2.waitForService(string clientHandle, float timeout)
map result = simROS2.call(string clientHandle, map request)
string serviceHandle = simROS2.createService(string serviceName, string serviceType, func serviceCallback)
simROS2.shutdownService(string serviceHandle)
simROS2.serviceTreatUInt8ArrayAsString(string serviceHandle)
string actionClientHandle = simROS2.createActionClient(string actionName, string actionType, func goalResponseCallback, func feedbackCallback, func resultCallback)
simROS2.shutdownActionClient(string actionClientHandle)
simROS2.actionClientTreatUInt8ArrayAsString(string actionClientHandle)
simROS2.spinSome()
bool success = simROS2.sendGoal(string actionClientHandle, map goal)
bool success = simROS2.cancelLastGoal(string actionClientHandle)
string actionServerHandle = simROS2.createActionServer(string actionName, string actionType, func handleGoalCallback, func handleCancelCallback, func handleAcceptedCallback)
simROS2.shutdownActionServer(string actionServerHandle)
simROS2.actionServerTreatUInt8ArrayAsString(string actionServerHandle)
simROS2.actionServerPublishFeedback(string actionServerHandle, string goalUUID, map feedback)
simROS2.actionServerActionAbort(string actionServerHandle, string goalUUID, map result)
simROS2.actionServerActionSucceed(string actionServerHandle, string goalUUID, map result)
simROS2.actionServerActionCanceled(string actionServerHandle, string goalUUID, map result)
simROS2.actionServerActionExecute(string actionServerHandle, string goalUUID)
bool result = simROS2.actionServerActionIsCanceling(string actionServerHandle, string goalUUID)
bool result = simROS2.actionServerActionIsActive(string actionServerHandle, string goalUUID)
bool result = simROS2.actionServerActionIsExecuting(string actionServerHandle, string goalUUID)
simROS2.sendTransform(map transform)
simROS2.sendTransforms(map transforms)
string subscriptionHandle = simROS2.imageTransportCreateSubscription(string topicName, func topicCallback, int queueSize=1)
simROS2.imageTransportShutdownSubscription(string subscriptionHandle)
string publisherHandle = simROS2.imageTransportCreatePublisher(string topicName, int queueSize=1)
simROS2.imageTransportShutdownPublisher(string publisherHandle)
simROS2.imageTransportPublish(string publisherHandle, buffer data, int width, int height, string frame_id)
map time = simROS2.getTime(int clock_type=simros2_clock_ros)
bool exists, string value = simROS2.getParamString(string name, string defaultValue="")
bool exists, int value = simROS2.getParamInt(string name, int defaultValue=0)
bool exists, float value = simROS2.getParamDouble(string name, float defaultValue=0.0)
bool exists, bool value = simROS2.getParamBool(string name, bool defaultValue=false)
simROS2.setParamString(string name, string value)
simROS2.setParamInt(string name, int value)
simROS2.setParamDouble(string name, float value)
simROS2.setParamBool(string name, bool value)
bool exists = simROS2.hasParam(string name)
simROS2.deleteParam(string name)
map result = simROS2.createInterface(string type)
map result = simROS2.getInterfaceConstants(string type)
string[] result = simROS2.supportedInterfaces()
simROS2.timeFromFloat(float t)
simROS2.timeToFloat(map t)
simROS2.getSystemTime()
simROS2.getSimulationTime()
simROS2.importInterface(string name)
simROS2.clock_type.ros
simROS2.clock_type.system
simROS2.clock_type.steady
simROS2.action_result_code.succeeded
simROS2.action_result_code.aborted
simROS2.action_result_code.canceled
simROS2.action_result_code.unknown
simROS2.goal_response.reject
simROS2.goal_response.accept_and_execute
simROS2.goal_response.accept_and_defer
simROS2.cancel_response.reject
simROS2.cancel_response.accept
simROS2.qos_history_policy.system_default
simROS2.qos_history_policy.keep_last
simROS2.qos_history_policy.keep_all
simROS2.qos_reliability_policy.system_default
simROS2.qos_reliability_policy.reliable
simROS2.qos_reliability_policy.best_effort
simROS2.qos_durability_policy.system_default
simROS2.qos_durability_policy.transient_local
simROS2.qos_durability_policy.volatile
simROS2.qos_liveliness_policy.system_default
simROS2.qos_liveliness_policy.automatic
simROS2.qos_liveliness_policy.manual_by_topic
]]

registerCodeEditorInfos("simROS2",codeEditorInfos)
