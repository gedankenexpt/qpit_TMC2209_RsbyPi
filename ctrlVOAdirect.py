import RPi.GPIO as GPIO
import time
import os
import numpy as np
import configparser
import pandas as pd
# import matplotlib.pyplot as plt


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
        GPIO.setup(pin, int(GPIO.IN), pull_up_down=GPIO.PUD_UP)


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


def make_steps_and_read(step_pin, num_steps, pin_ind_list, fpath=''):
    """
    method that makes a step
    """
    on_time = 80 * 1e-6
    off_time = 80 * 1e-6
    pause_time = 50 * 1e-6
    sig_index = np.zeros((2 * num_steps, len(pin_ind_list)), dtype=int)
    sig_index[:] = -1
    for j in range(num_steps):
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(on_time)
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(off_time)
        for ix, pin in enumerate(pin_ind_list):
            sig_index[j, ix] = GPIO.input(pin)

    for j in range(num_steps, 2 * num_steps):
        time.sleep(pause_time)
        for ix, pin in enumerate(pin_ind_list):
            sig_index[j, ix] = GPIO.input(pin)

    if len(fpath) > 0:
        print(f"Dumping index signal output in file {fpath}")
        df = pd.DataFrame(sig_index, columns=pin_ind_list)
        df.to_csv(fpath, index=False)
    else:
        print("Index signal output will not be dumped into file")


def plot_INDpin_values(data_src, pin_ind_list, ni=0):
    if len(os.path.split(data_src)[0]) > 0:
        data_src_is_file = False
        fig, ax = plt.subplots(nrows=ni, ncols=1, figsize=(10, 6))
    else:
        ni = 1  # force exactly one iteration
        data_src_is_file = True
        fig, ax = plt.subplots(figsize=(10, 6))

    # fig, ax = plt.subplots(figsize=(10, 6))

    for i in range(ni):
        print(f'Plotting from iteration {i + 1} of {ni}')
        if data_src_is_file:
            df = pd.read_csv(data_src)
        else:
            fpath_csv1 = os.path.join(data_src, 'CW' + str(i + 1) + '.csv')
            fpath_csv2 = os.path.join(data_src, 'CCW' + str(i + 1) + '.csv')
            df1 = pd.read_csv(fpath_csv1)
            df2 = pd.read_csv(fpath_csv2)

        for ix, pin in enumerate(pin_ind_list):
            offset = ix * 0.01
            if data_src_is_file:
                ax.plot(offset + df[pin],
                           label='IND' + str(ix + 1) + '(GPIO ' + pin)
                ax.legend(fontsize=12)
                ax.set_yticks([0, 1])
                ax.set_ylim([0, 1.1])
            else:
                ax[i].plot(offset + np.concatenate((df1[pin], df2[pin])),
                           label='IND' + str(ix + 1) + '(GPIO ' + pin + ')_run' + str(i + 1))
                ax[i].legend(fontsize=12)
                ax[i].set_yticks([0, 1])
                ax[i].set_ylim([0, 1.1])
                # ax[i].plot(df1[pin] + 1, label='CW' + pin + '(upper)')
                # col = ax[i].get_lines()[-1].get_color()
                # ax[i].plot(df2[pin] - 1, col, label='CCW' + pin + '(lower)')

    if data_src_is_file:
        ax.set_title(f'Data from {data_src}')
    else:
        ax[0].set_title(f'Data from {os.path.split(data_src)[1]}')

    return fig, ax


# def read_INDpins(pin_ind_list, num_vals, fname=''):
#     print(f'Reading a total of {num_vals} of index pins given by numbers {pin_ind_list}')
#     sig_index = np.zeros((num_vals, len(pin_ind_list)), dtype=int)
#     sig_index[:] = -1
#     for j in range(num_vals):
#         for ix, pin in enumerate(pin_ind_list):
#             sig_index[j, ix] = GPIO.input(pin)
#
#     if len(fname) > 0:
#         print(f"Dumping index signal output in file {fname}")
#         df = pd.DataFrame(sig_index, columns=pin_ind_list)
#         df.to_csv(fname, index=False)
#     else:
#         print("Index signal output will not be dumped into file")


def shutdown():
    print('Cleaning up GPIO: This will make all ports as INP')
    GPIO.cleanup()


def run_motor_direct(step_pin, dir_pin, en_pin, index_pins, voa_id, lin_range=0):
    datadir = 'ind-sig_voa=' + voa_id + '_steps=' + str(lin_range)
    if lin_range == 0:
        if os.path.exists(datadir):
            print(f'Path exists, doing nothing')
        else:
            os.mkdir(datadir)
            print(f'Directory created')
        fixed_loop_steps = 30000
        ni = 3
        for i in range(ni):
            print(f'Iteration {i + 1} of {ni} starting...')
            GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.HIGH)  # set enable pin high
            GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.HIGH)  # set dir pin high
            # make_steps(step_pin, fixed_loop_steps)  # make steps
            fpath_csv = os.path.join(datadir, 'CW' + str(i + 1) + '.csv')
            make_steps_and_read(step_pin, fixed_loop_steps, index_pins, fpath_csv)  # make steps and read index signal
            print(f'Setting enable pin to low, waiting for 2 sec, and changing direction')
            GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.LOW)  # set enable pin low
            time.sleep(0.5)
            GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.HIGH)  # set enable pin high
            GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.LOW)  # set dir pin high
            # make_steps(step_pin, fixed_loop_steps)  # make steps again (in opposite direction)
            fpath_csv = os.path.join(datadir, 'CCW' + str(i + 1) + '.csv')
            make_steps_and_read(step_pin, fixed_loop_steps, index_pins,
                                fpath_csv)  # make steps (in opposite direction) and read index signal
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
        fpath_csv = datadir + '.csv'  # just the single file
        make_steps_and_read(step_pin, int(abs(lin_range)), index_pins, fpath_csv)  # make steps and read index signal
        print(f'Setting enable pin to low, waiting for 1 sec')
        GPIO.setup(en_pin, GPIO.OUT, initial=GPIO.LOW)  # set enable pi low


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
    print(f'Setting GPIO pin {pin_en} to be the enable pin')
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
        run_motor_direct(pin_step, pin_dir, pin_en, ind_pins_all, cfg['voa']['id'], steps_to_move)
    except:
        print(f'Encountered some error. Closing down.')
    finally:
        shutdown()

    print("---")
    print("SCRIPT FINISHED")
    print("---")
