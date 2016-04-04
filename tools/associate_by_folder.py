#!/usr/bin/python
import argparse
import sys
import os
import numpy

def scan_images(folderPath):
    """
    Scan image path in the specified path

    Input:
        path

    Output:
        list -- the list of image path

    """
    extensions = {'.jpeg','.jpg', '.png', '.bmp'}
    images = []

    for root, dirs, files in os.walk(folderPath):
        for file in files:
            if file.lower().endswith(tuple(extensions)):
                relatviePath = os.path.join(root, file)
                images.append(relatviePath)
    return images

def get_list_from_folder(folderPath):
    """
    Scan image path and map the corresponding timestamp

    Input:
        path

    Output:
        dict -- dictionary of (stamp,data) tuples

    """
    imgs = scan_images(folderPath)
    ret = {}
    for img in imgs:
        timestamp = os.path.basename(img).split('.')[:-1][0].strip()
        ret[float(timestamp)] = img.strip()
    return ret

def read_file_list(filename):
    """
    Reads a trajectory from a text file.

    File format:
        The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
        and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.

    Input:
        filename -- File name
    Output:

        dict -- dictionary of (stamp,data) tuples

    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n")
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)

def associate(first_list, second_list,offset,max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim
    to find the closest match for every input tuple.

    Input:
        first_list -- first dictionary of (stamp,data) tuples
        second_list -- second dictionary of (stamp,data) tuples
        offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
        max_difference -- search radius for candidate generation

    Output:
        matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))

    """
    first_keys = first_list.keys()
    second_keys = second_list.keys()
    potential_matches = [(abs(a - (b + offset)), a, b)
                         for a in first_keys
                         for b in second_keys
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))

    matches.sort()
    return matches

if __name__ == '__main__':

    # parse command line
    parser = argparse.ArgumentParser(description='''
                                     This script takes two data files with timestamps and associates them
                                     ''')
    parser.add_argument('first_file', help='first text file (format: timestamp data)')
    parser.add_argument('second_file', help='second text file (format: timestamp data)')
    parser.add_argument('--first_only', help='only output associated lines from first file', action='store_true')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.002)
    args = parser.parse_args()

    first_list = get_list_from_folder(args.first_file)
    second_list = get_list_from_folder(args.second_file)

    matches = associate(first_list, second_list,float(args.offset),float(args.max_difference))

    if args.first_only:
        for a,b in matches:
            print("%f %s"%(a," ".join(first_list[a])))
    else:
        for a,b in matches:
            print("%f %s %f %s"%(a," " + first_list[a],b-float(args.offset)," " + second_list[b]))


