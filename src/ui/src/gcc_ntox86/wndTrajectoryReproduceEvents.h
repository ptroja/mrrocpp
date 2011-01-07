/* ../wndTrajectoryReproduceEvents.cc */
void SetButtonState(PtWidget_t *widget, short active);
int TRbtnStart(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnPause(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnStop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnExit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnPositionZero(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnLoadTrajectory(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int
TRbtnSaveAll(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRConnect(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRRefreshWindow(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnDSSCalibrate(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnFSCalibrate(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnTryAgain(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRDangerousForceDetected(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
