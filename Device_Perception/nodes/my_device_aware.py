#!/usr/bin/env python

# from asyncio.windows_events import NULL
import os
import math
import time
import re
import rospy

def get_cpu_freq():
    f = open('/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq', 'r')
    if(f):
        return round(int(f.read()) / 1000)
    else:
        return 0

def get_cpu_temp():
    f = open('/sys/class/thermal/thermal_zone0/temp', 'r')
    if(f):
        # f.seek(0)
        return round(int(f.read()) / 1000, 1)      # temperature need divide 1000
    else:
        return 0

def cpu_stat_process(stat_str):
    stat_str = stat_str.replace('  ', ' ')      # replace
    stat_str = ''.join(stat_str)                # implode
    stat_str = stat_str.split(' ')              # explode
    stat_dict = {}
    stat_dict['user'] = float(stat_str[1])
    stat_dict['nice'] = float(stat_str[2])
    stat_dict['sys'] = float(stat_str[3])
    stat_dict['idle'] = float(stat_str[4])
    stat_dict['iowait'] = float(stat_str[5])
    stat_dict['irq'] = float(stat_str[6])
    stat_dict['softirq'] = float(stat_str[7])
    return stat_dict

def get_file_info(file_address):
    # f = open(file_address, 'r')
    f = os.popen('cat /proc/stat')
    if(f):
        str_file = f.read()
        f.close()
        return str_file
    else:
        print("open file fail ", file_address)

def get_cpu_stat(str_dic_old):
    file_address = '/proc/stat'
    stat_str = get_file_info(file_address)
    # time.sleep(0.25)
    # get the process str
    if str_dic_old == None:
        str_dic = cpu_stat_process(stat_str)
        return str_dic, None
    # get new stat str
    else:
        # old is not None
        # get the info 
        str_dic = cpu_stat_process(stat_str)
        use_total = str_dic['idle'] + str_dic['user'] + str_dic['sys'] + str_dic['nice'] \
            + str_dic['iowait'] + str_dic['irq'] + str_dic['softirq'] \
            - str_dic_old['idle'] - str_dic_old['user'] - str_dic_old['sys'] \
            - str_dic_old['nice'] - str_dic_old['iowait'] - str_dic_old['irq'] - str_dic_old['softirq']
        if use_total != 0:
            cpu_stat_return = {}
            cpu_stat_return['Cpu_idle'] = round((str_dic['idle'] - str_dic_old['idle']) / use_total * 100, 2)
            cpu_stat_return['Cpu_user'] = round((str_dic['user'] - str_dic_old['user']) / use_total * 100, 2)
            cpu_stat_return['Cpu_sys'] = round((str_dic['sys'] - str_dic_old['sys']) / use_total * 100, 2)
            cpu_stat_return['Cpu_nicer'] = round((str_dic['nice'] - str_dic_old['nice']) / use_total * 100, 2)
            cpu_stat_return['Cpu_iowait'] = round((str_dic['iowait'] - str_dic_old['iowait']) / use_total * 100, 2)
            cpu_stat_return['Cpu_irq'] = round((str_dic['irq'] - str_dic_old['irq']) / use_total * 100, 2)
            cpu_stat_return['Cpu_softirq'] = round((str_dic['softirq'] - str_dic_old['softirq']) / use_total * 100, 2)
            return str_dic, cpu_stat_return
        else:
            return str_dic, None

def get_meminfo(str_mem_return):
    f = open('/proc/meminfo', 'r')
    if(f):
        str_meminfo = f.read()
        str_meminfo = str_meminfo.replace(' ', '').replace(':', ' ').replace('kB', '')
        str_meminfo = ''.join(str_meminfo)
        str_meminfo = str_meminfo.split()
        meminfo_dict = {}
        for i in range(len(str_meminfo)):
            if i % 2 == 0:
                meminfo_dict[str_meminfo[i]] =  str_meminfo[i+1]
            else:
                continue
        # print(meminfo_dict)
        # total free buffers cached cached_percent mem_used mem_percent mem_real_used mem_real_free 
        # swap_free swap_used swap_percent
        # Mem
        str_mem_return['MemTotal'] = round(int(meminfo_dict['MemTotal']) / 1024, 2)
        str_mem_return['MemFree'] = round(int(meminfo_dict['MemFree']) / 1024, 2)
        str_mem_return['Mem_used'] = round(str_mem_return['MemTotal'] - str_mem_return['MemFree'], 2)
        if str_mem_return['MemTotal'] != 0:
            str_mem_return['Mem_percent'] = round(str_mem_return['Mem_used'] / str_mem_return['MemTotal'] * 100, 2)
        else:
            str_mem_return['Mem_percent'] = 0
        # Buffer
        str_mem_return['Buffers'] = round(int(meminfo_dict['Buffers']) / 1024, 2)
        # Cache
        str_mem_return['Cached'] = round(int(meminfo_dict['Cached']) / 1024, 2)
        if str_mem_return['Cached'] != 0:
            str_mem_return['Cached_percent'] = round(str_mem_return['Cached'] / str_mem_return['MemTotal'] * 100, 2)
        else:
            str_mem_return['Cached_percent'] = 0
        # Real mem
        str_mem_return['Mem_real_used'] = round(str_mem_return['MemTotal'] - str_mem_return['MemFree'] - 
            str_mem_return['Cached'] - str_mem_return['Buffers'], 2)
        str_mem_return['Mem_real_free'] = round(str_mem_return['MemTotal'] - str_mem_return['Mem_real_used'], 2)
        if str_mem_return['MemTotal'] != 0:
            str_mem_return['Mem_real_percent'] = round(str_mem_return['Mem_real_used'] / str_mem_return['MemTotal'] * 100, 2)
        else:
            str_mem_return['Mem_real_percent'] = 0
        # Swap
        str_mem_return['Swap_total'] = round(int(meminfo_dict['SwapTotal']) / 1024, 2)
        str_mem_return['Swap_free'] = round(int(meminfo_dict['SwapFree']) / 1024, 2)
        str_mem_return['Swap_used'] = round(str_mem_return['Swap_total'] - str_mem_return['Swap_free'], 2)
        if str_mem_return['Swap_total'] != 0:
            str_mem_return['Swap_percent'] = round(str_mem_return['Swap_used'] / str_mem_return['Swap_total'] * 100, 2)
        else:
            str_mem_return['Swap_percent'] = 0
    else:
        # if meminfo is None
        str_mem_return['MemTotal'] = 0
        str_mem_return['MemFree'] = 0
        str_mem_return['Mem_used'] = 0
        str_mem_return['Mem_percent'] = 0
        # Buffer
        str_mem_return['Buffers'] = 0
        # Cache
        str_mem_return['Cached'] = 0
        str_mem_return['Cached_percent'] = 0
        # Real mem
        str_mem_return['Mem_real_used'] = 0
        str_mem_return['Mem_real_free'] = 0
        str_mem_return['Mem_real_percent'] = 0
        # Swap
        str_mem_return['Swap_total'] = 0
        str_mem_return['Swap_free'] = 0
        str_mem_return['Swap_used'] = 0
        str_mem_return['Swap_percent'] = 0
    return str_mem_return

if __name__ == '__main__':

    rospy.init_node('device_aware')
    cpu_stat_dic = None
    try:
        while(True):
            info_dict = {}
            info_dict['Freq'] = get_cpu_freq()
            info_dict['Temp'] = get_cpu_temp()
            info_dict = get_meminfo(info_dict)
            # get cpu stat
            if cpu_stat_dic == None:
                cpu_stat_dic, _ = get_cpu_stat(cpu_stat_dic)
                # continue
            else:
                # get the cpu stat info
                cpu_stat_dic, cpu_stat_dic_return = get_cpu_stat(cpu_stat_dic)
                if(cpu_stat_dic_return):
                    info_dict.update(cpu_stat_dic_return)
            info_dict = list(info_dict.items())
            info_dict.sort(key=lambda x:x[0], reverse=False)
            # print(info_dict)
            print("Press Ctrl-Z to interrupt\n")
            print "*" * 10 , 'Device Resource', "*" * 10
            for info in info_dict:
                # print(info, ': ', info_dict[info])
                if info[0] == 'Temp' :
                    print info[0], ':', info[1] , 'C'
                elif info[0] == 'Freq' :
                    print info[0], ':', info[1] , 'MHz'
                elif info[0] == 'Cached_percent' or info[0] == 'Mem_real_percent' or info[0] == 'Swap_percent' or info[0] == 'Mem_percent' \
                    or info[0] == 'Cpu_idle' or info[0] == 'Cpu_iowait' or info[0] == 'Cpu_irq' or info[0] == 'Cpu_nicer' or info[0] == 'Cpu_softirq' \
                    or info[0] == 'Cpu_sys' or info[0] == 'Cpu_user':
                    print info[0], ':', info[1] , '%'
                else:
                    print info[0], ':', info[1] , 'MB'
            print "*" * 10 , 'Device Resource', "*" * 10, '\n'
            time.sleep(1)
    except:
        print("Some error here")
        
            

    
    
    
