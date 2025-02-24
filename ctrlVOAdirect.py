import RPi.GPIO as GPIO
import time
from enum import Enum, IntEnum
import configparser


# required to do in the beginning to ensure the motors are not drawing too much current
def assert_ENpins_low(pin_en_list):
    GPIO.setmode(GPIO.BCM)
    for pin in pin_en_list:
        try:
            GPIO.setup(pin, int(GPIO.OUT), initial=int(GPIO.LOW))
        except RuntimeWarning:
            print(f'RuntimeWarning came from pin {pin}')


def make_INDpins_inp(pin_ind_list):
    GPIO.setmode(GPIO.BCM)
    for pin in pin_ind_list:
        GPIO.setup(pin, int(GPIO.IN))


def make_stepNdir_output(step_pin, dir_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(step_pin, int(GPIO.OUT))
    GPIO.setup(dir_pin, int(GPIO.OUT))


def make_steps(step_pin, num_steps):
    """
    method that makes a step
    """
    on_time = 100 * 1e-6
    off_time = 100 * 1e-6
    for j in range(num_steps):
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(on_time)
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(off_time)


def shutdown():
    print('Cleaning up GPIO: This will make all ports as INP')
    GPIO.cleanup()


def run_motor_direct(step_pin, dir_pin, en_pin, lin_range=0):
    if lin_range == 0:
        fixed_loop_steps = 40000
        ni = 2
        for i in range(ni):
            print(f'Iteration {i + 1} of {ni} starting...')
            GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.HIGH)  # set enable pin high
            GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.HIGH)  # set dir pin high
            make_steps(step_pin, fixed_loop_steps)  # make steps
            print(f'Setting enable pin to low, waiting for 2 sec, and changing direction')
            GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.LOW)  # set enable pin low
            time.sleep(0.5)
            GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.HIGH)  # set enable pin high
            GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.LOW)  # set dir pin high
            make_steps(step_pin, fixed_loop_steps)  # make steps again (in opposite direction)
            print(f'Iteration {i + 1} of {ni} finished; setting enable pin low')
            GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.LOW)  # set enable pin low
            time.sleep(1.2)
    else:
        GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.HIGH)  # set enable pin high
        print(f'Moving by {lin_range} steps')
        if lin_range > 0:
            print(f'Going the +ve way')
            GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.HIGH)  # set dir pin high
        else:
            print(f'Going the -ve way')
            GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.LOW)  # set dir pin low
        time.sleep(0.1)
        make_steps(step_pin, int(abs(lin_range)))
        print(f'Setting enable pin to low, waiting for 1 sec')
        GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.LOW)  # set enable pin low


if __name__ == "__main__":
    # get stuff through the config file
    cfg = configparser.ConfigParser(comment_prefixes='#', inline_comment_prefixes='#')
    cfg.read('ctrlVOA.conf')
    list_enable_pins = cfg['voa']['enables'].split(',')
    list_index_pins = cfg['voa']['indexes'].split(',')
    chosen_voa_ix = int(cfg['voa']['id']) - 1  # -1 so that index starts from 0
    en_pins_all = [int(pin) for pin in list_enable_pins]
    ind_pins_all = [int(pin) for pin in list_index_pins]
    print(f'Setting all EN pins to low (to prevent motors from drawing too much current)')
    assert_ENpins_low(en_pins_all)
    time.sleep(1)
    print(f'Setting all IND pins to be input')
    make_INDpins_inp(ind_pins_all)
    time.sleep(1)

    pin_en = int(list_enable_pins[chosen_voa_ix])  # GPIO pin for the chosen one!
    pin_step = int(cfg['common']['step'])
    pin_dir = int(cfg['common']['direction'])
    print(f'Setting STEP and DIR pins to be output')
    make_stepNdir_output(pin_step, pin_dir)

    try:
        steps_to_move = int(cfg['movement']['num_steps'])
    except ValueError:
        print(f'Configured number of steps should be an integer')
        steps_to_move = 0

    try:
        run_motor_direct(pin_step, pin_dir, pin_en, steps_to_move)
    except:
        print(f'Encountered some error. Closing down.')
    finally:
        shutdown()

    print("---")
    print("SCRIPT FINISHED")
    print("---")
