/* ../fun_r_irp6_polycrank.cc */
int EDP_polycrank_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_polycrank_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_polycrank_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int import_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int start_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int init_wnd_polycrank_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_polycrank_joints_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int polycrank_int_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_polycrank_int_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int EDP_polycrank_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_polycrank_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

