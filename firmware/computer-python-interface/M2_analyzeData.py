import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import seaborn as sns
import timeit
from scipy import signal

sns.set()
sns.set_style("whitegrid")
plt.style.use('bmh')
# BEWARE: Currently in this version we assume for both Vicon and Wheelbot an
# SAMPLE FREQUENCY of 100 Hz!

#flag_analysis = 'IMU_acc'
#flag_analysis = 'IMU_rate'
flag_analysis = 'debug'
#flag_analysis = 'pivot_acc'
#flag_analysis = 'control_analysis'
#flag_analysis = 'noise'
#flag_analysis = 'scatter'
#flag_analysis = 'bias'

# READ: WHEELBOT DATA

#df = pd.read_csv('out.csv', encoding='utf-8')
df = pd.read_csv('out.csv', encoding='utf-8')

df = df[df['crc2']==0]
marker_flag = False

tstart = 0 # out12
tend = 0  # out12
sampling_rate = 100

df = df.set_index('time step')
df.index = df.index/sampling_rate
#df = df.rename(columns={'Roll': 'Roll (Wheelbot)', 'Pitch': 'Pitch (Wheelbot)'})

# START PLOTTING

# if flag_analysis == 'subplot':
#     fig, ax = plt.subplots(3, 3)
#
#     sns.lineplot(data=df[['Roll']], ci=None, ax=ax[0,0], palette=['red'], markers=True, markeredgecolor=None)    #
#     sns.lineplot(data=0.01*df[['Roll rate']], ci=None, ax=ax[0,0], palette=['salmon'], markers=True, markeredgecolor=None)
#     #sns.lineplot(data=df[['rate_m1']], ci=None, ax=ax[1,0], palette=['silver'], markers=True, markeredgecolor=None)
#     sns.lineplot(data=df[['I_sent1']], ci=None, ax=ax[2,0], palette=['burlywood'], markers=True, markeredgecolor=None)
#
#     sns.lineplot(data=df[['Pitch']], ci=None, ax=ax[0,1], palette=['blue'], markers=True, markeredgecolor=None)
#     #sns.lineplot(data=df[['Pitch rate']], ci=None, ax=ax[0,1], palette=['steelblue'], markers=True, markeredgecolor=None)
#     #sns.lineplot(data=df[['rate_m2']], ci=None, ax=ax[1,1], palette=['violet'], markers=True, markeredgecolor=None)
#     sns.lineplot(data=df[['I_sent2']], ci=None, ax=ax[2,1], palette=['maroon'], markers=True, markeredgecolor=None)
#
#     #sns.lineplot(data=df[['battery_voltage']], ci=None, ax=ax[0,2], palette=['yellow'], markers=True, markeredgecolor=None)
#
#     #sns.lineplot(data=df[['debug1']], ci=None, ax=ax[0,2], palette=['green'], markers=True, markeredgecolor=None)
#     #sns.lineplot(data=df[['debug2']], ci=None, ax=ax[0,2], palette=['lightgreen'], markers=True, markeredgecolor=None)
#     #sns.lineplot(data=df[['debug3']], ci=None, ax=ax[0,2], palette=['olive'], markers=True, markeredgecolor=None)
#
#     sns.lineplot(data=df[['crc2']], ci=None, ax=ax[0,2], palette=['green'], markers=True, markeredgecolor=None)
#
#     plt.xlabel('t [s]', fontsize=12)
#     plt.ylabel('value', fontsize=12)
#     plt.show()

if flag_analysis == 'IMU_acc':
    fig, ax = plt.subplots(4, 1, sharex = True)

    sns.lineplot(data=df[['acc_imuA_x']], ci=None, ax=ax[0], markers=True, markeredgecolor=None, palette=['indianred'])
    sns.lineplot(data=df[['acc_imuA_y']], ci=None, ax=ax[0], markers=True, markeredgecolor=None, palette=['steelblue'])
    sns.lineplot(data=df[['acc_imuA_z']], ci=None, ax=ax[0], markers=True, markeredgecolor=None, palette=['mediumseagreen'])

    sns.lineplot(data=df[['acc_imuB_x']], ci=None, ax=ax[1], markers=True, markeredgecolor=None, palette=['indianred'])
    sns.lineplot(data=df[['acc_imuB_y']], ci=None, ax=ax[1], markers=True, markeredgecolor=None, palette=['steelblue'])
    sns.lineplot(data=df[['acc_imuB_z']], ci=None, ax=ax[1], markers=True, markeredgecolor=None, palette=['mediumseagreen'])

    sns.lineplot(data=df[['acc_imuC_x']], ci=None, ax=ax[2], markers=True, markeredgecolor=None, palette=['indianred'])
    sns.lineplot(data=df[['acc_imuC_y']], ci=None, ax=ax[2], markers=True, markeredgecolor=None, palette=['steelblue'])
    sns.lineplot(data=df[['acc_imuC_z']], ci=None, ax=ax[2], markers=True, markeredgecolor=None, palette=['mediumseagreen'])

    sns.lineplot(data=df[['acc_imuD_x']], ci=None, ax=ax[3], markers=True, markeredgecolor=None, palette=['indianred'])
    sns.lineplot(data=df[['acc_imuD_y']], ci=None, ax=ax[3], markers=True, markeredgecolor=None, palette=['steelblue'])

    plt.xlabel('t [s]', fontsize=12)
    plt.show()

if flag_analysis == 'IMU_rate':
    fig, ax = plt.subplots(4, 1, sharex=True)

    sns.lineplot(data=df[['rate_imuA_x']], ci=None, ax=ax[0], markers=True, markeredgecolor=None, palette=['indianred'])
    sns.lineplot(data=df[['rate_imuA_y']], ci=None, ax=ax[0], markers=True, markeredgecolor=None, palette=['steelblue'])
    sns.lineplot(data=df[['rate_imuA_z']], ci=None, ax=ax[0], markers=True, markeredgecolor=None, palette=['mediumseagreen'])

    sns.lineplot(data=df[['rate_imuB_x']], ci=None, ax=ax[1], markers=True, markeredgecolor=None, palette=['indianred'])
    sns.lineplot(data=df[['rate_imuB_y']], ci=None, ax=ax[1], markers=True, markeredgecolor=None, palette=['steelblue'])
    sns.lineplot(data=df[['rate_imuB_z']], ci=None, ax=ax[1], markers=True, markeredgecolor=None, palette=['mediumseagreen'])

    sns.lineplot(data=df[['rate_imuC_x']], ci=None, ax=ax[2], markers=True, markeredgecolor=None, palette=['indianred'])
    sns.lineplot(data=df[['rate_imuC_y']], ci=None, ax=ax[2], markers=True, markeredgecolor=None, palette=['steelblue'])
    sns.lineplot(data=df[['rate_imuC_z']], ci=None, ax=ax[2], markers=True, markeredgecolor=None, palette=['mediumseagreen'])

    sns.lineplot(data=df[['rate_imuD_x']], ci=None, ax=ax[3], markers=True, markeredgecolor=None, palette=['indianred'])
    sns.lineplot(data=df[['rate_imuD_y']], ci=None, ax=ax[3], markers=True, markeredgecolor=None, palette=['steelblue'])

    plt.xlabel('t [s]', fontsize=12)
    plt.show()

if flag_analysis == 'debug':
    fig, ax = plt.subplots()

    #sns.lineplot(data=df[['Roll']], ci=None, ax=ax, palette=['red'], markers=True, markeredgecolor=None)    #
    #sns.lineplot(data=df[['Roll rate']], ci=None, ax=ax, palette=['salmon'], markers=True, markeredgecolor=None)
    #sns.lineplot(data=0.01*df[['rate_m1']], ci=None, ax=ax, palette=['silver'], markers=True, markeredgecolor=None)
    #sns.lineplot(data=df[['I_sent1']], ci=None, ax=ax, palette=['burlywood'], markers=True, markeredgecolor=None)

    sns.lineplot(data=df[['Pitch']], ci=None, ax=ax, palette=['blue'], markers=True, markeredgecolor=None)
    #sns.lineplot(data=df[['Pitch rate']], ci=None, ax=ax, palette=['steelblue'], markers=True, markeredgecolor=None)
    sns.lineplot(data=0.01*df[['rate_m2']], ci=None, ax=ax, palette=['violet'], markers=True, markeredgecolor=None)
    sns.lineplot(data=df[['I_sent2']], ci=None, ax=ax, palette=['maroon'], markers=True, markeredgecolor=None)

    #sns.lineplot(data=df[['battery_voltage']], ci=None, palette=['yellow'], markers=True, markeredgecolor=None)

    #sns.lineplot(data=df[['debug1']], ci=None, ax=ax, palette=['green'], markers=True, markeredgecolor=None)
    #sns.lineplot(data=df[['debug2']], ci=None, ax=ax, palette=['lightgreen'], markers=True, markeredgecolor=None)
    sns.lineplot(data=df[['debug3']], ci=None, ax=ax, palette=['olive'], markers=True, markeredgecolor=None)

    #sns.lineplot(data=df[['crc2']], ci=None, ax=ax, palette=['green'], markers=True, markeredgecolor=None)


    #plt.xlabel('t [s]', fontsize=12)
    #plt.ylabel('value', fontsize=12)
    plt.show()

if flag_analysis == 'pivot_acc':
    fig, ax = plt.subplots()

    #sns.lineplot(data=df[['Pitch']], ci=None, ax=ax, palette=['blue'], markers=True, markeredgecolor=None)
    #sns.lineplot(data=df[['Pitch rate']], ci=None, ax=ax, palette=['steelblue'], markers=True, markeredgecolor=None)
    #sns.lineplot(data=df[['rate_m2']], ci=None, ax=ax, palette=['violet'], markers=True, markeredgecolor=None)
   # sns.lineplot(data=df[['I_sent2']], ci=None, ax=ax, palette=['maroon'], markers=True, markeredgecolor=None)

    sns.lineplot(data=df[['debug1']], ci=None, ax=ax, palette=['green'], markers=True, markeredgecolor=None)
    sns.lineplot(data=df[['debug2']], ci=None, ax=ax, palette=['lightgreen'], markers=True, markeredgecolor=None)
    #sns.lineplot(data=df[['debug3']], ci=None, ax=ax, palette=['olive'], markers=True, markeredgecolor=None)

    #df['Pitch_shifted'] = df['Pitch'].shift(periods=5, fill_value=0)
    df['diff'] = df['debug1'] - df['debug2']
    sns.lineplot(data=df[['diff']], ci=None, ax=ax, palette=['red'], markers=True, markeredgecolor=None)
    #sns.lineplot(data=df[['Pitch_shifted']], ci=None, ax=ax, palette=['red'], markers=True, markeredgecolor=None)

    # Real wheel speed
    df['dq4'] = df['debug3']- df['Pitch rate']
    dq4 = df[['dq4']].to_numpy()
    b, a = signal.butter(4, 0.3, analog=False)
    dq4_ff = signal.filtfilt(b, a, dq4[:, 0])
    times = df.index.to_numpy()
    #plt.plot(times, dq4_ff, color='silver', label='dq4_ff')

    # Used wheel speed
    rate_m2 = df[['rate_m2']].to_numpy()
    rate_m2_ff = signal.filtfilt(b, a, rate_m2[:, 0])

    # Compute Acc
    ddq4_numerical = np.gradient(dq4_ff)
    dd_rate_m2_ff = np.gradient(rate_m2_ff)

    plt.plot(times, 0.5*ddq4_numerical, color='black', label='ddq4')
    #plt.plot(times, dd_rate_m2_ff, color='blue', label='acc_rate')

    #sns.lineplot(data=df[['dq4']], ci=None, ax=ax, palette=['steelblue'], markers=True, markeredgecolor=None)


    #plt.xlabel('t [s]', fontsize=12)
    #plt.ylabel('value', fontsize=12)
    plt.show()

if flag_analysis == 'control_analysis':
    """
    Compare computed torque on wheelbot with computed torque in python
    To use this script set flag_debug = 2 and ensure that the measured current is comm_ud.currentMotor
    """
    roll_gains = [4.4 , 0.31, 0.0001, 0.00195]
    pitch_gains = [1.6, 0.14, 0.04, 0.0344]

    rpm2rads = np.pi / 30
    deg2rad = np.pi/180
    torque2current = 1/0.075

    df['I_roll'] = torque2current * roll_gains[0] * deg2rad * df[['Roll']]
    df['I_roll_rate'] = torque2current * roll_gains[1] * df[['Roll rate']]
    df['I_wheel1_pos'] = torque2current * roll_gains[2] * 0.0 * df[['debug2']]
    df['I_wheel1_rate'] = torque2current * roll_gains[3] * rpm2rads * df[['rate_m1']]

    df['I_total_roll'] =  df['I_roll'] + df['I_roll_rate'] + df['I_wheel1_rate'] + df['I_wheel1_pos']
    df['I_diff_roll'] = df['I_total_roll'] - df['I_sent1']

    df['I_pitch'] = torque2current * pitch_gains[0] * deg2rad * df[['Pitch']]
    df['I_pitch_rate'] = torque2current * pitch_gains[1] * df[['Pitch rate']]
    df['I_wheel2_pos'] = torque2current * pitch_gains[2] * 1.0 * df[['debug3']]
    df['I_wheel2_rate'] = torque2current * pitch_gains[3] * rpm2rads * df[['rate_m2']]

    df['I_total_pitch'] =  df['I_pitch'] + df['I_pitch_rate'] + df['I_wheel2_rate'] + df['I_wheel2_pos']
    df['I_diff_pitch'] = df['I_total_pitch'] - df['I_sent2']

    #df_slice = df[48.5:]
    #print(np.deg2rad(df_slice['Roll'].mean()))

    fig, ax = plt.subplots()
    #sns.lineplot(data= df[['debug1']], ci=None, ax=ax, palette=['red'], markers=True, markeredgecolor=None, legend='full')
    #sns.lineplot(data=df[['Roll']], ci=None, ax=ax, palette=['rosybrown'], markers=True, markeredgecolor=None, legend='full')    #

    #sns.lineplot(data=df[['I_roll']], ci=None, ax=ax, palette=['red'], markers=True, markeredgecolor=None, legend='full')    #
    #sns.lineplot(data=df[['I_roll_rate']], ci=None, ax=ax, palette=['salmon'], markers=True, markeredgecolor=None, legend='full')
    #sns.lineplot(data=df[['I_wheel1_pos']], ci=None, ax=ax, palette=['grey'], markers=True, markeredgecolor=None, legend='full')
    #sns.lineplot(data=df[['I_wheel1_rate']], ci=None, ax=ax, palette=['silver'], markers=True, markeredgecolor=None, legend='full')
    #sns.lineplot(data=df[['I_sent1']], ci=None, ax=ax, palette=['burlywood'], markers=True, markeredgecolor=None, legend='full')
    #sns.lineplot(data= df[['I_total_roll']], ci=None, ax=ax, palette=['green'], markers=True, markeredgecolor=None, legend='full')
    #sns.lineplot(data= df[['I_diff_roll']], ci=None, ax=ax, palette=['blue'], markers=True, markeredgecolor=None, legend='full')

    sns.lineplot(data=df[['I_pitch']], ci=None, ax=ax, palette=['red'], markers=True, markeredgecolor=None, legend='full')    #
    sns.lineplot(data=df[['I_pitch_rate']], ci=None, ax=ax, palette=['salmon'], markers=True, markeredgecolor=None, legend='full')
    #sns.lineplot(data=df[['I_wheel2_pos']], ci=None, ax=ax, palette=['grey'], markers=True, markeredgecolor=None, legend='full')
    sns.lineplot(data=df[['I_wheel2_rate']], ci=None, ax=ax, palette=['silver'], markers=True, markeredgecolor=None, legend='full')
    #sns.lineplot(data=df[['I_sent2']], ci=None, ax=ax, palette=['burlywood'], markers=True, markeredgecolor=None, legend='full')
    #sns.lineplot(data= df[['I_total_pitch']], ci=None, ax=ax, palette=['green'], markers=True, markeredgecolor=None, legend='full')
    #sns.lineplot(data= df[['I_diff_pitch']], ci=None, ax=ax, palette=['blue'], markers=True, markeredgecolor=None, legend='full')

    #plt.legend(loc='upper right', labels=['u <- roll', 'u <- roll_rate', 'u <- m1_pos', 'u <- m1_rate']) # title='Smoker',
    #plt.xlabel('t [s]', fontsize=12)
    plt.ylabel('Current', fontsize=12)
    plt.show()

if flag_analysis == 'noise':
    # Scatterplot
    sns.lmplot(x='Roll', y='Pitch', data=df,
               fit_reg=False) # No regression line
               #hue='Roll')   # Color by evolution stag
    plt.show()

    # Boxplot
    sns.boxplot(data=df)
    plt.show()

    # Violin plot
    sns.violinplot(data=df['Roll'])
    plt.show()

if flag_analysis == 'scatter':
    fig, ax = plt.subplots(1,1)
    lim_val = np.max(np.abs(df[['I_sent1']]))

    df.plot.scatter(x='rate_m1', y = 'Roll', c = 'I_sent1', colormap = 'Spectral', ax=ax, zorder=2, vmin=-lim_val, vmax=lim_val) # bwr hsv 'viridis'
    df.plot.line(x='rate_m1', y='Roll', color='lightgray', ax=ax, zorder=1)
    #df.plot.line(x='rate_m1', y = 'Roll', ax=ax, style='b')
    #plt.grid()
    plt.show()

    fig, ax = plt.subplots(1, 1)
    #lim_val = np.max(np.abs(df[['I_sent2']]))
    lim_val=5

    df.plot.scatter(x='rate_m2', y='Pitch', c='I_sent2', colormap='Spectral', ax=ax, zorder=2, vmin=-lim_val,
                    vmax=lim_val)  # bwr hsv 'viridis'
    df.plot.line(x='rate_m2', y='Pitch', color='lightgray', ax=ax, zorder=1)
    # df.plot.line(x='rate_m1', y = 'Roll', ax=ax, style='b')
    # plt.grid()
    plt.show()

if flag_analysis == 'bias':
    slice_start = 28
    slice_end =60
    clip = 10

    if slice_end == 0:
        df_truncated = df[slice_start:]
    else:
        df_truncated = df[slice_start:slice_end]
    #df_truncated = df_truncated[ df_truncated < clip ]
    #df_truncated = df_truncated[ df_truncated > -clip ]

    fig, ax = plt.subplots()

    sns.lineplot(data=df_truncated[['Roll']], ci=None, ax=ax, palette=['red'], markers=True, markeredgecolor=None)    #
    sns.lineplot(data=df_truncated[['Pitch']], ci=None, ax=ax, palette=['blue'], markers=True, markeredgecolor=None)

    roll_mean = np.mean(df_truncated['Roll'])
    pitch_mean = np.mean(df_truncated['Pitch'])

    plt.axhline(y=roll_mean, color='r', linestyle='-')
    plt.axhline(y=pitch_mean, color='b', linestyle='-')

    print('roll_mean in rad : ', np.deg2rad(roll_mean))
    print('pitch_mean in rad: ', np.deg2rad(pitch_mean))
    #plt.xlabel('t [s]', fontsize=12)
    #plt.ylabel('value', fontsize=12)

    #
    #roll_mean in rad: -0.010795381105239055
    #pitch_mean in rad: 0.0036408923408705633

    #roll_mean in rad: -0.012355374805888424
    #pitch_mean in rad: 0.01725571090312542

    plt.show()



