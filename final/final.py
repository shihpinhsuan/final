import sensor, image, time, pyb, math

enable_lens_corr = False # turn on for straighter lines...

sensor.reset()

sensor.set_pixformat(sensor.RGB565) # grayscale is faster

sensor.set_framesize(sensor.QQVGA)

sensor.skip_frames(time = 2000)

sensor.set_auto_gain(False)  # must turn this off to prevent image washout...

sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...

clock = time.clock()

f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set

f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set

c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)

c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)


def degrees(radians):

   return (180 * radians) / math.pi

uart = pyb.UART(3,9600,timeout_char=1000)

uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)

# All lines also have `x1()`, `y1()`, `x2()`, and `y2()` methods to get their end-points

# and a `line()` method to get all the above as one 4 value tuple for `draw_line()`.


while(True):

   clock.tick()

   img = sensor.snapshot()

   count = 0

   if enable_lens_corr: img.lens_corr(1.8) # for 2.8mm lens...


   # `merge_distance` controls the merging of nearby lines. At 0 (the default), no

   # merging is done. At 1, any line 1 pixel away from another is merged... and so

   # on as you increase this value. You may wish to merge lines as line segment

   # detection produces a lot of line segment results.


   # `max_theta_diff` controls the maximum amount of rotation difference between

   # any two lines about to be merged. The default setting allows for 15 degrees.


   for l in img.find_line_segments(merge_distance = 0, max_theta_diff = 5):

 #     img.draw_line(l.line(), color = (255, 0, 0))

      # print(l)

       if l.y1()<50 and l.y2() <50 and count == 0:
           count = count + 1
           img.draw_line(l.line(), color = (255, 0, 0))
           print_line = (l.x1(), l.x2(), l.y1(), l.y2())
           uart.write(("l%3d%3d%3d%3d" %print_line).encode())
           print("l%3d %3d %3d %3d" %print_line)


   for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11

      img.draw_rectangle(tag.rect(), color = (255, 0, 0))

      img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))

      # The conversion is nearly 6.2cm to 1 -> translation

      print_args = (100 * tag.x_translation(), 100 * tag.y_translation(), 100 * tag.z_translation(), \

            degrees(tag.x_rotation()), degrees(tag.y_rotation()), degrees(tag.z_rotation()))

      # Translation units are unknown. Rotation units are in degrees.

      uart.write(("s%4d%4d%4d%3d%3d%3d" % print_args).encode())

      print("s%4d %4d %4d %3d %3d %3d" % print_args)

   count = 0;
