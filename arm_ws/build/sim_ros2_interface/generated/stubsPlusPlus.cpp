
// include this file after calling the SIM_PLUGIN(...) macro

SIM_DLLEXPORT void simROS2_createSubscription(createSubscription_in *in, createSubscription_out *out)
{
    sim::plugin->createSubscription(in, out);
}
SIM_DLLEXPORT void simROS2_shutdownSubscription(shutdownSubscription_in *in, shutdownSubscription_out *out)
{
    sim::plugin->shutdownSubscription(in, out);
}
SIM_DLLEXPORT void simROS2_subscriptionTreatUInt8ArrayAsString(subscriptionTreatUInt8ArrayAsString_in *in, subscriptionTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->subscriptionTreatUInt8ArrayAsString(in, out);
}
SIM_DLLEXPORT void simROS2_createPublisher(createPublisher_in *in, createPublisher_out *out)
{
    sim::plugin->createPublisher(in, out);
}
SIM_DLLEXPORT void simROS2_shutdownPublisher(shutdownPublisher_in *in, shutdownPublisher_out *out)
{
    sim::plugin->shutdownPublisher(in, out);
}
SIM_DLLEXPORT void simROS2_publisherTreatUInt8ArrayAsString(publisherTreatUInt8ArrayAsString_in *in, publisherTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->publisherTreatUInt8ArrayAsString(in, out);
}
SIM_DLLEXPORT void simROS2_publish(publish_in *in, publish_out *out)
{
    sim::plugin->publish(in, out);
}
SIM_DLLEXPORT void simROS2_createClient(createClient_in *in, createClient_out *out)
{
    sim::plugin->createClient(in, out);
}
SIM_DLLEXPORT void simROS2_shutdownClient(shutdownClient_in *in, shutdownClient_out *out)
{
    sim::plugin->shutdownClient(in, out);
}
SIM_DLLEXPORT void simROS2_clientTreatUInt8ArrayAsString(clientTreatUInt8ArrayAsString_in *in, clientTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->clientTreatUInt8ArrayAsString(in, out);
}
SIM_DLLEXPORT void simROS2_waitForService(waitForService_in *in, waitForService_out *out)
{
    sim::plugin->waitForService(in, out);
}
SIM_DLLEXPORT void simROS2_call(call_in *in, call_out *out)
{
    sim::plugin->call(in, out);
}
SIM_DLLEXPORT void simROS2_createService(createService_in *in, createService_out *out)
{
    sim::plugin->createService(in, out);
}
SIM_DLLEXPORT void simROS2_shutdownService(shutdownService_in *in, shutdownService_out *out)
{
    sim::plugin->shutdownService(in, out);
}
SIM_DLLEXPORT void simROS2_serviceTreatUInt8ArrayAsString(serviceTreatUInt8ArrayAsString_in *in, serviceTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->serviceTreatUInt8ArrayAsString(in, out);
}
SIM_DLLEXPORT void simROS2_createActionClient(createActionClient_in *in, createActionClient_out *out)
{
    sim::plugin->createActionClient(in, out);
}
SIM_DLLEXPORT void simROS2_shutdownActionClient(shutdownActionClient_in *in, shutdownActionClient_out *out)
{
    sim::plugin->shutdownActionClient(in, out);
}
SIM_DLLEXPORT void simROS2_actionClientTreatUInt8ArrayAsString(actionClientTreatUInt8ArrayAsString_in *in, actionClientTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->actionClientTreatUInt8ArrayAsString(in, out);
}
SIM_DLLEXPORT void simROS2_spinSome(spinSome_in *in, spinSome_out *out)
{
    sim::plugin->spinSome(in, out);
}
SIM_DLLEXPORT void simROS2_sendGoal(sendGoal_in *in, sendGoal_out *out)
{
    sim::plugin->sendGoal(in, out);
}
SIM_DLLEXPORT void simROS2_cancelLastGoal(cancelLastGoal_in *in, cancelLastGoal_out *out)
{
    sim::plugin->cancelLastGoal(in, out);
}
SIM_DLLEXPORT void simROS2_createActionServer(createActionServer_in *in, createActionServer_out *out)
{
    sim::plugin->createActionServer(in, out);
}
SIM_DLLEXPORT void simROS2_shutdownActionServer(shutdownActionServer_in *in, shutdownActionServer_out *out)
{
    sim::plugin->shutdownActionServer(in, out);
}
SIM_DLLEXPORT void simROS2_actionServerTreatUInt8ArrayAsString(actionServerTreatUInt8ArrayAsString_in *in, actionServerTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->actionServerTreatUInt8ArrayAsString(in, out);
}
SIM_DLLEXPORT void simROS2_actionServerPublishFeedback(actionServerPublishFeedback_in *in, actionServerPublishFeedback_out *out)
{
    sim::plugin->actionServerPublishFeedback(in, out);
}
SIM_DLLEXPORT void simROS2_actionServerActionAbort(actionServerActionAbort_in *in, actionServerActionAbort_out *out)
{
    sim::plugin->actionServerActionAbort(in, out);
}
SIM_DLLEXPORT void simROS2_actionServerActionSucceed(actionServerActionSucceed_in *in, actionServerActionSucceed_out *out)
{
    sim::plugin->actionServerActionSucceed(in, out);
}
SIM_DLLEXPORT void simROS2_actionServerActionCanceled(actionServerActionCanceled_in *in, actionServerActionCanceled_out *out)
{
    sim::plugin->actionServerActionCanceled(in, out);
}
SIM_DLLEXPORT void simROS2_actionServerActionExecute(actionServerActionExecute_in *in, actionServerActionExecute_out *out)
{
    sim::plugin->actionServerActionExecute(in, out);
}
SIM_DLLEXPORT void simROS2_actionServerActionIsCanceling(actionServerActionIsCanceling_in *in, actionServerActionIsCanceling_out *out)
{
    sim::plugin->actionServerActionIsCanceling(in, out);
}
SIM_DLLEXPORT void simROS2_actionServerActionIsActive(actionServerActionIsActive_in *in, actionServerActionIsActive_out *out)
{
    sim::plugin->actionServerActionIsActive(in, out);
}
SIM_DLLEXPORT void simROS2_actionServerActionIsExecuting(actionServerActionIsExecuting_in *in, actionServerActionIsExecuting_out *out)
{
    sim::plugin->actionServerActionIsExecuting(in, out);
}
SIM_DLLEXPORT void simROS2_sendTransform(sendTransform_in *in, sendTransform_out *out)
{
    sim::plugin->sendTransform(in, out);
}
SIM_DLLEXPORT void simROS2_sendTransforms(sendTransforms_in *in, sendTransforms_out *out)
{
    sim::plugin->sendTransforms(in, out);
}
SIM_DLLEXPORT void simROS2_imageTransportCreateSubscription(imageTransportCreateSubscription_in *in, imageTransportCreateSubscription_out *out)
{
    sim::plugin->imageTransportCreateSubscription(in, out);
}
SIM_DLLEXPORT void simROS2_imageTransportShutdownSubscription(imageTransportShutdownSubscription_in *in, imageTransportShutdownSubscription_out *out)
{
    sim::plugin->imageTransportShutdownSubscription(in, out);
}
SIM_DLLEXPORT void simROS2_imageTransportCreatePublisher(imageTransportCreatePublisher_in *in, imageTransportCreatePublisher_out *out)
{
    sim::plugin->imageTransportCreatePublisher(in, out);
}
SIM_DLLEXPORT void simROS2_imageTransportShutdownPublisher(imageTransportShutdownPublisher_in *in, imageTransportShutdownPublisher_out *out)
{
    sim::plugin->imageTransportShutdownPublisher(in, out);
}
SIM_DLLEXPORT void simROS2_imageTransportPublish(imageTransportPublish_in *in, imageTransportPublish_out *out)
{
    sim::plugin->imageTransportPublish(in, out);
}
SIM_DLLEXPORT void simROS2_getTime(getTime_in *in, getTime_out *out)
{
    sim::plugin->getTime(in, out);
}
SIM_DLLEXPORT void simROS2_getParamString(getParamString_in *in, getParamString_out *out)
{
    sim::plugin->getParamString(in, out);
}
SIM_DLLEXPORT void simROS2_getParamInt(getParamInt_in *in, getParamInt_out *out)
{
    sim::plugin->getParamInt(in, out);
}
SIM_DLLEXPORT void simROS2_getParamDouble(getParamDouble_in *in, getParamDouble_out *out)
{
    sim::plugin->getParamDouble(in, out);
}
SIM_DLLEXPORT void simROS2_getParamBool(getParamBool_in *in, getParamBool_out *out)
{
    sim::plugin->getParamBool(in, out);
}
SIM_DLLEXPORT void simROS2_setParamString(setParamString_in *in, setParamString_out *out)
{
    sim::plugin->setParamString(in, out);
}
SIM_DLLEXPORT void simROS2_setParamInt(setParamInt_in *in, setParamInt_out *out)
{
    sim::plugin->setParamInt(in, out);
}
SIM_DLLEXPORT void simROS2_setParamDouble(setParamDouble_in *in, setParamDouble_out *out)
{
    sim::plugin->setParamDouble(in, out);
}
SIM_DLLEXPORT void simROS2_setParamBool(setParamBool_in *in, setParamBool_out *out)
{
    sim::plugin->setParamBool(in, out);
}
SIM_DLLEXPORT void simROS2_hasParam(hasParam_in *in, hasParam_out *out)
{
    sim::plugin->hasParam(in, out);
}
SIM_DLLEXPORT void simROS2_deleteParam(deleteParam_in *in, deleteParam_out *out)
{
    sim::plugin->deleteParam(in, out);
}
SIM_DLLEXPORT void simROS2_createInterface(createInterface_in *in, createInterface_out *out)
{
    sim::plugin->createInterface(in, out);
}
SIM_DLLEXPORT void simROS2_getInterfaceConstants(getInterfaceConstants_in *in, getInterfaceConstants_out *out)
{
    sim::plugin->getInterfaceConstants(in, out);
}
SIM_DLLEXPORT void simROS2_supportedInterfaces(supportedInterfaces_in *in, supportedInterfaces_out *out)
{
    sim::plugin->supportedInterfaces(in, out);
}

