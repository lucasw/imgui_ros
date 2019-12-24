uint8 PUB=1
uint8 SUB=2
uint8 PLOT=3
uint8 PARAM=4
uint8 GRAPH=5
uint8 DYNREC=6
# pub sub types
uint8 BOOL=0
uint8 FLOAT32=1
uint8 FLOAT64=2
uint8 INT8=3
uint8 INT16=4
uint8 INT32=5
uint8 INT64=6
uint8 UINT8=7
uint8 UINT16=8
uint8 UINT32=9
uint8 UINT64=10
uint8 STRING=11
uint8 MENU=12
uint8 TF=13
uint8 VIZ2D=14
uint8 VIZ3D=15
uint8 POINTCLOUD=16
uint8 IMAGE=17
uint8 ROSOUT=18
# uint8 TODO(lucasw)
string name  # ""
# TODO(lucasw) maybe window_name belongs here too?
# Or get tab name out of here and push up to AddWindow
# currently Widget is within AddWindow, but could refactor that.
string tab_name  # ""
string topic  # ""
uint8 type  # 1  # PUB
uint8 sub_type  # 1  # FLOAT32
bool remove  # False
float64 value  # 0.0
float64 min  # 0.0
float64 max # 1.0
bool enable_info  # True  # draw extra misc info, controls
string[] items
