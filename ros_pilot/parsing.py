import sys
import string
import re

### main() starts here ###

file_name = sys.argv[1]
file_name2 = sys.argv[2]
f = open(file_name,'r')
fd = open(file_name2,'w')

lines = f.readlines()
list_line = []
list_line2 = []
main_list = []
temp_list = []
check_line = 0
check_line2=0
bag = 0
#### Write function() ####
for line in lines:
	if line.startswith('#line'):
		continue
	if line.startswith('int test__start_main(void)'):
		list_line.append("void main()")
	else :
	 	list_line.append(line)
	 	continue

	 
	 
"""
	if check_line == 0:
		if line.startswith('[value] Values at end of function orb_publish:'):
			list_line.append(line)
			check_line = 1
			continue
	if check_line == 1:
	  	if line.startswith('[value] Values at end of function orb_set_interval:'):
			check_line = 2
			continue
		else :
			if check_line == 1:
				list_line.append(line)
			continue
"""
"""
for line in lines:
	if check_line2 == 0:
		if line.startswith('[value] Values at end of function px4_poll:'):
			list_line2.append(line)
			check_line2 = 1
			continue
	if check_line2 == 1:
	  	if line.startswith('[value] Values at end of function radians:'):
			check_line2 = 2
			continue
		else :
		 	if check_line2 == 1:
		 		list_line2.append(line)
		 	continue
"""
for line in list_line:
	main_list.append(line)

for line in list_line2:
	main_list.append(line)

for line in main_list:
	  	fd.write(line)
	
#print('\n\n' + '#'*20 +' Print data for VL '+ '#' *20 + '\n\n')


### print data for VL###
#for strs in main_list:
#
#	if strs.startswith('  wind_estimate_size'):
#		wind_estimate_topic = strs[2:15]
#		wind_estimate_size = strs[29:31]
#		print ('topic_name   : '+wind_estimate_topic+'\t\t  topic_size : '+wind_estimate_size)
#	if strs.startswith('  vehicle_local_position_size'):
#	print strs
#		print ('topic_name   : '+strs[2:24]+'\t  topic_size : '+strs[38:41])
#		print strs[38:41]
#	if strs.startswith('  vehicle_global_position_size'):
#		print strs
#		print ('topic_name   : '+strs[2:25]+'\t  topic_size : '+strs[39:41])
#		print strs[39:41]
#	if strs.startswith('  vehicle_attitude_size'):
#		print strs
#		print ('topic_name   : '+strs[2:18]+'\t\t  topic_size : '+strs[32:35])
#	if strs.startswith('  estimator_status_size'):
#		print strs
#		print ('topic_name   : '+strs[2:18]+'\t\t  topic_size : '+strs[32:35])
#	if strs.startswith('  interval'):
		
#		print ('bag_interval : '+strs[16:19]+'\n\n')
	
