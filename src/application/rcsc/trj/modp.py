import math, sys 

value0 = float(sys.argv[1]) 
value1 = float(sys.argv[2]) 
value2 = float(sys.argv[3]) 
value3 = float(sys.argv[4]) 
value4 = float(sys.argv[5]) 
value5 = float(sys.argv[6]) 

output0 = value0
output1 = value1
output2 = value2 - output1 - math.pi/2
output3 = value3 - output2 - output1 - math.pi/2
output4 = value4
output5 = value5

print '%g %g %g %g %g %g' % (output0, output1, output2, output3, output4, output5) 

