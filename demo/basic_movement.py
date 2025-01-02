# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import
# pylint: disable=unused-import
# pylint: disable=duplicate-code
"""
test file for testing basic movement
"""

import configparser
import argparse
import sys
import time

try:
    from src.TMC_2209.TMC_2209_StepperDriver import *
    from src.TMC_2209._TMC_2209_GPIO_board import Board
except ModuleNotFoundError:
    from TMC_2209.TMC_2209_StepperDriver import *
    from TMC_2209._TMC_2209_GPIO_board import Board

if __name__ == "__main__":

    # get stuff through the config file
    cfg = configparser.ConfigParser(comment_prefixes='#', inline_comment_prefixes='#')
    cfg.read('basic_movement.conf')
    list_enable_pins = cfg['voa']['enables'].split(',')

    chosen_voa_ix = int(cfg['voa']['id']) - 1  # -1 so that index starts from 0
    if len(list_enable_pins) != 4 or chosen_voa_ix > 3 or chosen_voa_ix < 0:
        print(f'ERROR: Check config parameters')
        sys.exit(-1)

    en_pins = [int(pin) for pin in list_enable_pins]
    pin_en = en_pins[chosen_voa_ix]
    pin_step = int(cfg['common']['step'])
    pin_dir = int(cfg['common']['direction'])
    try:
        steps_to_move = int(cfg['movement']['num_steps'])
    except ValueError:
        print(f'Configured number of steps should be an integer')
        steps_to_move = 0

    print(f'GPIO pin {pin_en} will be enabled to move VOA{chosen_voa_ix + 1}')
    time.sleep(2)

    # -----------------------------------------------------------------------
    # initiate the TMC_2209 class
    # use your pins for pin_en, pin_step, pin_dir here
    # -----------------------------------------------------------------------
    if BOARD == Board.RASPBERRY_PI:
        tmc = TMC_2209(pin_en, pin_step, pin_dir, loglevel=Loglevel.DEBUG)
    elif BOARD == Board.RASPBERRY_PI5:
        tmc = TMC_2209(21, 16, 20, serialport="/dev/ttyAMA0", loglevel=Loglevel.DEBUG)
    elif BOARD == Board.NVIDIA_JETSON:
        tmc = TMC_2209(22, 6, 5, serialport="/dev/ttyTHS1", loglevel=Loglevel.DEBUG)
    else:
        # just in case
        tmc = TMC_2209(pin_en, pin_step, pin_dir, loglevel=Loglevel.DEBUG)

    # -----------------------------------------------------------------------
    # set the loglevel of the libary (currently only printed)
    # set whether the movement should be relative or absolute
    # both optional
    # -----------------------------------------------------------------------
    tmc.tmc_logger.set_loglevel(Loglevel.DEBUG)
    tmc.set_movement_abs_rel(MovementAbsRel.ABSOLUTE)

    # -----------------------------------------------------------------------
    # these functions change settings in the TMC register
    # -----------------------------------------------------------------------
    tmc.set_direction_reg(False)
    tmc.set_current(300)
    tmc.set_interpolation(True)
    tmc.set_spreadcycle(False)  # False will activate Stealthchop
    tmc.set_microstepping_resolution(2)
    tmc.set_internal_rsense(False)

    print("---\n---")

    # -----------------------------------------------------------------------
    # these functions read and print the current settings in the TMC register
    # -----------------------------------------------------------------------
    tmc.read_ioin()
    tmc.read_chopconf()
    tmc.read_drv_status()
    tmc.read_gconf()

    print("---\n---")

    # -----------------------------------------------------------------------
    # set the Acceleration and maximal Speed
    # -----------------------------------------------------------------------
    # tmc.set_acceleration(2000)
    # tmc.set_max_speed(500)

    # -----------------------------------------------------------------------
    # set the Acceleration and maximal Speed in fullsteps
    # -----------------------------------------------------------------------
    tmc.set_acceleration_fullstep(1000)
    tmc.set_max_speed_fullstep(250)

    # -----------------------------------------------------------------------
    # Get various parameters before enabling the motor
    # -----------------------------------------------------------------------
    time.sleep(0.2)
    dir_reg = tmc.get_direction_reg()
    interp = tmc.get_interpolation()
    resol_microstep_drv_reg = tmc.read_microstepping_resolution()
    resol_microstep_cached = tmc.get_microstepping_resolution()
    accel = tmc.get_acceleration()
    max_speed = tmc.get_acceleration()
    print(f'get_direction_reg() = {dir_reg}')
    print(f'get_interpolation() = {interp}')
    print(f'read_microstepping_resolution() = {resol_microstep_drv_reg}')
    print(f'get_microstepping_resolution() = {resol_microstep_cached}')
    print(f'get_acceleration() = {accel}')
    print(f'get_acceleration() = {max_speed}')

    # -----------------------------------------------------------------------
    # activate the motor current output
    # -----------------------------------------------------------------------
    tmc.set_motor_enabled(True)
    # -----------------------------------------------------------------------
    # move the motor the desired number of steps
    # -----------------------------------------------------------------------
    if steps_to_move:
        print(f'Moving by {steps_to_move} steps')
        tmc.run_to_position_steps(steps_to_move, MovementAbsRel.RELATIVE)
    else:
        for i in range(4):
            print(f'Iteration {i} starting...')
            tmc.run_to_position_steps(800, MovementAbsRel.RELATIVE)  # move 600 steps forward
            tmc.run_to_position_steps(-800, MovementAbsRel.RELATIVE)  # move 600 steps backward
            print(f'Iteration {i} finished, waiting for 2 sec')
            time.sleep(2)

    # -----------------------------------------------------------------------
    # deactivate the motor current output
    # -----------------------------------------------------------------------
    tmc.set_motor_enabled(False)

    print("---\n---")

    # -----------------------------------------------------------------------
    # deinitiate the TMC_2209 class
    # -----------------------------------------------------------------------
    del tmc

    print("---")
    print("SCRIPT FINISHED")
    print("---")
