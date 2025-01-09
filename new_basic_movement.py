from src.TMC_2209.TMC_2209_StepperDriver import *
# from src.TMC_2209._TMC_2209_GPIO_board import Board
import time


def set_motor(tmc, res_mstep):
    tmc.set_direction_reg(False)
    tmc.set_current(300)
    tmc.set_interpolation(True)
    tmc.set_spreadcycle(False)
    tmc.set_microstepping_resolution(res_mstep)
    tmc.set_internal_rsense(False)
    tmc.set_acceleration_fullstep(1000)
    tmc.set_max_speed_fullstep(250)

def run_motor(tmc, lin_range=0):
    tmc.set_motor_enabled(True)
    if lin_range == 0:
        ni = 3
        for i in range(ni):
            print(f'Iteration {i+1} of {ni} starting...')
            tmc.run_to_position_steps(9000, MovementAbsRel.RELATIVE)
            tmc.run_to_position_steps(-9000, MovementAbsRel.RELATIVE)
            print(f'Iteration {i+1} of {ni} finished, waiting for 1 sec')
            time.sleep(1)
    else:
        print(f'Moving by {lin_range} steps')
        tmc.run_to_position_steps(lin_range, MovementAbsRel.RELATIVE)
    tmc.set_motor_enabled(False)
    print(f'Pausing for 1 sec')
    time.sleep(1)


for ix in range(3, 4, 1):
    micro_step_res = 2**(ix + 1)
    print('\n------------------------------------------------')
    print(f'Setting micro step resolution to {micro_step_res}')
    print('------------------------------------------------\n')

    # VOA 1
    #print(f'Enabling VOA1 at pin 4')
    print(f'Enabling VOA1 at UART addr 3 (enable pin = 4)')
    tmc1 = TMC_2209([1], 5, 6, driver_address=3)
    set_motor(tmc1, micro_step_res)
    run_motor(tmc1, 0)
    del tmc1

    # VOA 2
    print(f'Enabling VOA2 at UART addr 2 (enable pin = 22)')
    tmc2 = TMC_2209([22], 5, 6, driver_address=2)
    set_motor(tmc2, micro_step_res)
    run_motor(tmc2, 0)
    del tmc2

    # VOA 3
    print(f'Enabling VOA3 at UART addr 0 (enable pin = 0)')
    tmc3 = TMC_2209([0], 5, 6, driver_address=0)
    set_motor(tmc3, micro_step_res)
    run_motor(tmc3, 0)
    del tmc3

    # VOA 4
    print(f'Enabling VOA4 at UART addr 1 (enable pin = 4)')
    tmc4 = TMC_2209([4], 5, 6, driver_address=1)
    set_motor(tmc4, micro_step_res)
    run_motor(tmc4, 0)
    del tmc4


print(f'Out of loop')
