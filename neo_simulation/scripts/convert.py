from tf.transformations import quaternion_from_euler

   
if __name__ == '__main__':
 
   # RPY to convert: 90deg, 0, -90deg
   q = quaternion_from_euler(0.000, 0.000, 1.5708)
 
   print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
