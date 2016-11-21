# The gains for joints and body feedback controllers 

float64[6]            kp_body   # AX, AY, AZ, LX, LY, LZ
float64[6]            kd_body

float64[12]           kp_joint  # LF_HAA, LF_HFE, LF_KFE, RF_..., LH_..., RH_...
float64[12]           kd_joint