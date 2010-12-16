
/* ../wndForceControlEvents.cc */
int FCwndForceControlRealised(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCCreateConnection(void);
int FCRefreshPosition(void);
int FCTimerTick(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnCalibrateSensor(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnAddMacrostep(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnNewTrajectory(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnSaveTrajectory(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnExit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnOnOffReader(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnChangeExternalMotorControl(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
void SendMoveCommand(int move_type);
int FCbtnMove0Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove1Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove2Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove3Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove4Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove5Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove0Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove1Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove2Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove3Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove4Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove5Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
