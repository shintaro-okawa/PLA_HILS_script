# SCALEXIO
import udfrt  # Common sequences
import Variables  # Variable in scripts settings
# RTT standard library functions.
from rttlib.utilities import Wait  # Instraction to wait sequence. waiting period is defined by second.
from rttlib.utilities import currentTime  # Get currenttime from HILS connection was start.


# ******************************************************************************
# Run directly from RTT . Create a variable object and run main
# ******************************************************************************
def MainGenerator(*args):
    v = void_class()
    yield udfrt.SetVariableForUdfrt(v)
    yield Variables.SetVar(v)
    yield Initialize(v)  # Set initial condition    
    yield main(v)  # Main script
    yield Initialize(v)  # Return to initial condition


# ******************************************************************************
# Set initial condition
# ******************************************************************************
def Initialize(v):

    # Speed initialization
    spd = (v.VSA_ABS_FR_WHEEL_SPEED.Value + v.VSA_ABS_FL_WHEEL_SPEED.Value + v.VSA_ABS_RR_WHEEL_SPEED.Value + v.VSA_ABS_RL_WHEEL_SPEED.Value) / 4
    spd = 0

    # Accel pedal initialization
    v.Flag_Accel_ON.Value = 0  # Accel pedal control activation falg
    v.Accel_Pedal_Input.Value = 0  # Accel pedal Value

    # Brake pedal initilization
    v.Flag_Brake_ON.Value = 0  # Brake pedal control activation falg
    v.Brake_Pedal_Input.Value = 0  # Brake pedal Value

    # Steering control initialization
    v.Flag_Steer_ON.Value = 0  # Steering angle control activation falg
    v.Steer_Angle_Input.Value = 0  # Steering angle Value

    # Shift position initialization
    v.Shift_Position.Value = 6  # Shift position (6:P,4→3:D,)

    # Road surface mue Values (at each wheels)
    v.Flag_Mue_ON.Value = 0  # Road mue control activation flag
    v.Mue_FL.Value = 1
    v.Mue_RL.Value = 1
    v.Mue_FR.Value = 1
    v.Mue_RR.Value = 1

    # Logging status (0:stop, 1:can only, 2:all signals)
    v.F_Measure.Value = 0

    # Road gradient control initialization
    v.Flag_Road_Gradient_ON.Value = 0  # Road gradient control activation falg
    v.Road_Gradient_Input.Value = 0  # Road gradient Value

    # Seat belt status
    v.SRS_DR_BELT_STATUS.Value = 2

    # EPB switch positions
    v.Return_SW.Value = 0  # Relase position (0 means released, 1 means pressed/pulled)
    v.LockUp_SW.Value = 0  # Apply position (0 means released, 1 means pressed/pulled)

    while v.V_S1.Value > 0.01:
        yield None

    yield udfrt.ReadDTC(v, disp=False, dispLog=False)
    if v.nDTC > 0:
        yield udfrt.SendCommand(v, "14 FF FF FF")
        v.Remote_IG_in.Value = 0
        yield Wait(3.0)

    if v.Remote_IG_in.Value == 0:
        v.Remote_IG_in.Value = 1
        yield Wait(4.0)

    yield udfrt.ReadDTC(v, disp=False, dispLog=False)
    if v.nDTC > 0:
        yield udfrt.ReadDTC(v, disp=False, dispLog=True)

    if v.EPB_STATE.Value == 3:
        v.Flag_Brake_ON.Value = 1
        v.Brake_Pedal_Input.Value = v.BF_3MPa
        yield Wait(2.0)
        v.Return_SW.Value = 1
        yield Wait(0.5)
        v.Return_SW.Value = 0
        v.Flag_Brake_ON.Value = 0
        v.Brake_Pedal_Input.Value = 0
        yield Wait(3.0)


# ******************************************************************************
# main
# ******************************************************************************
def main(v):
    XX = 0

    # ************************ Precondition ************************

    # *** Action ***
    # Set Gradient to 0km/h with no Brake
    v.Road_Gradient_Input.Value = 15 # must check value
    # Set brake pressure at vehicle stopping with under PLA target puressure.
    v.Flag_Brake_ON.Value = 1
    v.Brake_Pedal_Input.Value = 100 #we must define about Driver brake pressure
    yield Wait(1.0)

    #shift to D range
    v.Shift_Position.Value = 3

    # add EPB Apply request from EPB SW

    yield Wait(1.0)

    # *** Result check ***


    # ************************ Condition 1 ************************

    # *** Action ***
    yield Wait(1.0)

    # What hidden shift position means?
    # request lockup1 SW wait 0.5 lockup0
    v.Shift_Position.Value = 6
    yield Wait(1.0)

    # EPB Apply request from SBW
    v.IMG_REQUEST_EPB_ACT_MAC.Value = 1
    # difine the state that the car is stopped
    spd = (v.VSA_ABS_FR_WHEEL_SPEED.Value + v.VSA_ABS_FL_WHEEL_SPEED.Value + v.VSA_ABS_RR_WHEEL_SPEED.Value + v.VSA_ABS_RL_WHEEL_SPEED.Value) / 4

    # *** Result check ***
    if (v.EPB_STATE.Value == 1 and v.EPB_REQ_CTRL_PCA.Value == 1 and v.EPB_REQ_BRKTRQ_PCA.Value <= 100 and v.EPB_REQ_BRKTRQ_PCA.Value > v.BRK_DRV_REQ_TRQ_MAC.Value
 and v.ESB_PLA_ACT_MAC.Value == 1 and spd == 0):
        print("Condition 1 Result OK")

    else:
        print("Condition 1 Result NG")
    # ************************ Condition 2 ************************

    # *** Action ***

    # Wait for EPB appliying completion
    temp_time = currentTime
    while v.EPB_STATUS.Value != 2 and currentTime - temp_time < 10:
        pass

    # set EPB complete timing
    yield Wait(0.5)
    
    v.Shift_Position.Value = 6

    spd = (v.VSA_ABS_FR_WHEEL_SPEED.Value + v.VSA_ABS_FL_WHEEL_SPEED.Value + v.VSA_ABS_RR_WHEEL_SPEED.Value + v.VSA_ABS_RL_WHEEL_SPEED.Value) / 4
    # *** Result check ***
    if (v.EPB_STATE.Value == 3 and v.EPB_REQ_CTRL_PCA.Value == 1 and v.EPB_REQ_BRKTRQ_PCA.Value >= 100 and v.EPB_REQ_BRKTRQ_PCA.Value > v.BRK_DRV_REQ_TRQ_MAC.Value and v.ESB_PLA_ACT_MAC.Value == 1 and spd == 0):
        print("Condition 2 Result OK")
    else:
        print("Condition 2 Result NG")

    # ************************ Condition 3 ************************ 

    # *** Action ***
    temp_time = currentTime
    while v.EPB_REQ_BRKTRQ_PCA.Value == 0 and currentTime - temp_time < 10:
        pass

    yield Wait(0.5) 

    if (v.EPB_STATE.Value == 3 and v.EPB_REQ_CTRL_PCA.Value == 0 and v.EPB_REQ_BRKTRQ_PCA.Value == 0 and v.EPB_REQ_BRKTRQ_PCA.Value < v.BRK_DRV_REQ_TRQ_MAC.Value and v.ESB_PLA_ACT_MAC.Value == 0 and v.VSA_ABS_xy_WHEEL_SPEED.Value == 0):
        print("Condition 3 Result OK")
    else:
        print("Condition 3 Result NG")


class void_class():
    pass
