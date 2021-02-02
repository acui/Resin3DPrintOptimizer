#!/usr/bin/env python
# coding: utf-8

# In[52]:

import numpy as np
import cv2
import zipfile
import os.path
import decimal
import re
import math
import datetime

# In[2]:


def cv2np(image):
    rgbimage = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    return rgbimage / 255.


# In[3]:


def cvb2np(image):
    rgbimage = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    return rgbimage / 1.


# In[49]:

decimal.getcontext().prec = int(8)

# In[5]:


def read_nova3d_bene4_mono(ifs):
    img = cv2.imdecode(
        np.asarray(bytearray(ifs.read()), dtype=np.uint8), cv2.IMREAD_COLOR)
    shape = img.shape
    img.resize(shape[0], shape[1] * 3)
    return img


# In[6]:


def get_circle_kernel(radius):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                     (radius * 2 + 1, radius * 2 + 1))


# In[7]:


def get_bounding_box(img):
    rs, cs = img.nonzero()
    return (np.min(rs), np.max(rs) + 1, np.min(cs), np.max(cs) + 1)


# In[132]:


class Bene4MonoOptimizer:
    HEADER_PARAMETERS = re.compile(";\\({0,1}([^=]+)=([^=)]+)\\){0,1}")
    OPERATOR = re.compile(
        "((?:[A-Za-z0-9]+)|(?:<[A-Za-z0-9]+>)|(?:;<[A-Za-z0-9]+>))((?:\\s+(?:[A-Za-z0-9.-]+))*)((?:\\s+(?:;.*))*)"
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.cwsfile = None
        self.files = None
        self.support = None
        self.thickness = -1
        self.max_thickness = 5000
        self.max_speed = -1
        self.max_speed_area = int(300 * 300)
        self.min_speed = int(6)
        self.current_layer = 0
        self.done = False
        self.kernel = get_circle_kernel(10)
        self.speeds = []
        self.images = []
        self.gcode_headers = []
        self.lift_distance = -1
        self.wait_time = -1
        self.bottom_time = -1
        self.bottom_lift = -1
        self.bottom_down = -1
        self.bottom_count = -1
        self.normal_time = -1
        self.normal_lift = -1
        self.normal_down = -1
        self.normal_count = -1
        self.resolution_x = -1
        self.resolution_y = -1
        self.current_read_layer = -1
        self.machine_status = 'init'
        self.base_name = ''

    def optimize(self, filename, outputfilename):
        self.reset()
        with open(filename, 'rb') as ifs:
            self.cwsfile = zipfile.ZipFile(ifs)
            self.files = sorted(
                self.cwsfile.filelist, key=lambda info: info.filename)
            for file in self.files:
                if os.path.splitext(file.filename)[1].upper() == ".GCODE":
                    self.gcode_filename = file.filename
                    self._read_gcode_file(file)
                    break
            self.support_layers = self.max_thickness / self.thickness
            self.max_speed = self.normal_down
            for file in self.files:
                if os.path.splitext(file.filename)[1].upper() == ".PNG":
                    self.speeds.append(self._calculate_layer(file))
            self.done = True
            self.base_name = os.path.splitext(filename)[0]
            if outputfilename is None:
                print(self.gen_output())
            else:
                self._write(outputfilename, self.cwsfile)

    def _write(self, outputfilename, source):
        output_filename = outputfilename
        with zipfile.ZipFile(output_filename, 'w',
                             zipfile.ZIP_DEFLATED) as zfile:
            with zfile.open(self.gcode_filename, 'w') as ofs:
                ofs.write(self.gen_output().encode('utf-8'))
            for file in self.files:
                if file.filename != self.gcode_filename:
                    with zfile.open(file.filename, 'w') as ofs:
                        ofs.write(source.open(file, 'r').read())

    def _read_gcode_file(self, file_info):
        with self.cwsfile.open(file_info, 'r') as ifs:
            status = 'header'
            line = ifs.readline()
            while line:
                line = line.decode('utf-8')
                if status == 'header':
                    m = self.OPERATOR.match(line)
                    if m is not None:
                        status = 'body'
                        self.current_read_layer = 0
                        self.handle_operator(m)
                    else:
                        self.gcode_headers.append(line)
                        m = self.HEADER_PARAMETERS.match(line)
                        if m is not None:
                            self.handle_header_parameter(m[1], m[2])
                elif status == 'body':
                    m = self.OPERATOR.match(line)
                    if m is not None:
                        self.handle_operator(m)
                line = ifs.readline()

    def get_length(self, value):
        return math.floor(decimal.Decimal(value[:-2].strip()) * int(1000))

    def get_time(self, value):
        if value.endswith('ms'):
            return int(value[:-2].strip())
        elif value.endswith('s'):
            return int(value[:-1].strip()) * int(1000)
        return None

    def get_speed(self, value):
        if value.endswith('mm/s'):
            return demical.Decimal(value[:-4].strip())

    def handle_header_parameter(self, key, value):
        key = key.strip()
        value = value.strip()
        if key == 'Pix per mm X':
            self.ppm_x = decimal.Decimal(value)
        elif key == 'Pix per mm Y':
            self.ppm_y = decimal.Decimal(value)
        elif key == 'X Resolution':
            self.resolution_x = int(value)
        elif key == 'Y Resolution':
            self.resolution_y = int(value)
        elif key == 'Layer Thickness':
            self.thickness = self.get_length(value)
        elif key == 'Layer Time':
            self.normal_time = self.get_time(value)
        elif key == 'Bottom Layers Time':
            self.bottom_time = self.get_time(value)
        elif key == 'Number of Bottom Layers':
            self.bottom_count = int(value)
        elif key == 'Lift Distance':
            self.lift_distance = self.get_length(value)
        elif key == 'Wait After Stop':
            self.wait_time = self.get_time(value)
        elif key == 'Number of Slices':
            self.layers = int(value)

    def handle_operator(self, m):
        key = m[1].strip()
        if self.machine_status == 'init':
            if key == "M106":
                value = m[2].strip()
                if value == "S0":
                    self.machine_status = "printing"
        elif self.machine_status == 'printing':
            if key == "M106":
                value = m[2].strip()
                if value == "S0":
                    self.machine_status = "moving"
            elif key == "M18":
                self.machine_status = "finishing"
        elif self.machine_status == 'moving':
            if key == ';<Delay>':
                self.machine_status = "printing"
                self.current_read_layer += 1
            elif key == 'G1':
                values = m[2].strip().split()
                height = int(decimal.Decimal(values[0][1:]) * int(1000))
                speed = int(values[1][1:])
                if self.current_read_layer < self.bottom_count:
                    if height > 0:
                        if self.bottom_lift < 0:
                            self.bottom_lift = speed
                    else:
                        if self.bottom_down < 0:
                            self.bottom_down = speed
                else:
                    if height > 0:
                        self.normal_lift = max(self.normal_lift, speed)
                    else:
                        self.normal_down = max(self.normal_down, speed)
        elif self.machine_status == 'finishing':
            if key == ';<Completed>':
                self.machine_status = 'finished'

    def gen_output(self):
        total_time = 0
        result = ""
        for header in self.gcode_headers:
            result += header
        result += '\n'

        result += """G28
G21 ;Set units to be mm
G91 ;Relative Positioning
M17 ;Enable motors
<Slice> Blank
M106 S0
"""
        result += '\n'
        bottom_move_delay = math.ceil(
            (self.lift_distance * int(60) / self.bottom_lift +
             (self.lift_distance - self.thickness) * int(60) / self.bottom_down
             + self.wait_time) / int(1000)) * int(1000)
        for i in range(self.bottom_count):
            result += """;<Slice> {layer}
M106 S255
;<Delay> {bottom_time}
M106 S0
;<Slice> Blank
G1 Z{lift_distance:0.3f} F{bottom_lift}
G1 Z{down_distance:0.3f} F{bottom_down}
;<Delay> {bottom_move_delay}
""".format(
                **{
                    'layer':
                    i,
                    'bottom_time':
                    self.bottom_time,
                    'lift_distance':
                    decimal.Decimal(self.lift_distance) / int(1000),
                    'down_distance':
                    -decimal.Decimal(self.lift_distance - self.thickness) /
                    int(1000),
                    'bottom_lift':
                    self.bottom_lift,
                    'bottom_down':
                    self.bottom_down,
                    'bottom_move_delay':
                    bottom_move_delay
                })
            result += '\n'
            total_time += self.bottom_time + bottom_move_delay

        for i in range(self.bottom_count, self.layers):
            speed = max(self.min_speed, int(self.speeds[i]))
            move_delay = math.ceil(
                (self.lift_distance * int(60) / speed +
                 (self.lift_distance - self.thickness) * int(60) / speed +
                 self.wait_time) / int(1000)) * int(1000)
            result += """;<Slice> {layer}
M106 S255
;<Delay> {normal_time}
M106 S0
;<Slice> Blank
G1 Z{lift_distance:0.3f} F{speed}
G1 Z{down_distance:0.3f} F{speed}
;<Delay> {move_delay}
""".format(
                **{
                    'layer':
                    i,
                    'normal_time':
                    self.normal_time,
                    'lift_distance':
                    decimal.Decimal(self.lift_distance) / int(1000),
                    'down_distance':
                    -decimal.Decimal(self.lift_distance - self.thickness) /
                    int(1000),
                    'speed':
                    speed,
                    'move_delay':
                    move_delay
                })
            result += '\n'
            total_time += self.normal_time + move_delay

        result += """M18 ;Disable Motors
M106 SO
G1 Z80 F{speed}
;<Completed>
""".format(speed=self.normal_lift)
        total_time = datetime.timedelta(milliseconds=total_time)
        print(";total time: {time}".format(time=str(total_time)))
        return result

    def _calculate_layer(self, file_info):
        img = read_nova3d_bene4_mono(self.cwsfile.open(file_info, 'r'))
        if self.support is None:
            self.support = np.ones(img.shape, dtype=np.int32) * int(255)
        self.images.append(img)
        areas, labels = cv2.connectedComponents(img, connectivity=int(4))
        new_support = np.zeros(img.shape, dtype=np.int32)
        speed = self.max_speed
        for i in range(1, areas):
            area = (labels == i)
            bbox = get_bounding_box(area)
            area = area[bbox[0]:bbox[1], bbox[2]:bbox[3]]

            area_speed = self.max_speed * self.max_speed_area / len(
                area.nonzero()[0])
            area_support = self.support[bbox[0]:bbox[1], bbox[2]:bbox[3]] * area
            support_max = np.amax(area_support)
            area_newsupport = area_support + area
            rs, cs = (area_newsupport > support_max).nonzero()
            area_newsupport[rs, cs] = support_max
            new_support[bbox[0]:bbox[1], bbox[2]:bbox[3]] += area_newsupport
            max_support = np.max(area_support)
            if max_support >= self.support_layers:
                has_support = area_support >= self.support_layers
                distance_speed = self.max_speed
            else:
                has_support = area_support >= max_support
                distance_speed = self.max_speed * max_support / self.support_layers
            if not np.any(has_support):
                print("warning: no support")
            area_test = np.array(area, dtype=np.uint8)
            radius = int(100)
            has_support = cv2.dilate(
                np.array(has_support, dtype=np.uint8),
                self.kernel,
                iterations=int(5)) * area
            area_test -= has_support * area_test
            while (np.any(area_test > 0)):
                has_support = cv2.dilate(has_support, self.kernel) * area
                scale = float(radius) / float(radius + int(10))
                distance_speed = distance_speed * (scale**int(4))
                area_test -= has_support * area_test
                radius += 10
            speed = min(speed, area_speed, distance_speed)
            # print("file: " + str(file_info.filename) + "\tarea: " + str(i) + "\tspeed: " + str(speed) + "\tarea_speed: " + str(area_speed) + "\tdistance_speed: " + str(distance_speed))
        self.support = new_support
        self.current_layer += 1
        print(";done processing " + str(self.current_layer))
        return speed


# In[146]:


def draw_image(img):
    img = np.array(img, dtype=np.uint8)
    matrix_plot(cv2np(img)).show(
        axes=True, frame=True, figsize=8, aspect_ratio=1)


# In[198]:


def draw_boolean_image(img):
    img = np.array(img, dtype=np.uint8)
    matrix_plot(cvb2np(img)).show(
        axes=True, frame=True, figsize=8, aspect_ratio=1)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description=
        'Optimize the lift speed of each layers. If no output file is given, will print the gcode.'
    )
    parser.add_argument(
        'input',
        metavar='input',
        type=str,
        help='file that need to be optimized')
    parser.add_argument(
        '-o',
        '--output',
        metavar='output',
        default=None,
        type=str,
        help='optimized file')
    args = parser.parse_args()
    o = Bene4MonoOptimizer()
    o.optimize(args.input, args.output)
