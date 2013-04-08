#!/usr/bin/env python

from survey_route_plan import route_plan 

file_name = 'opmaaling_motorvej.txt'
import_dir = 'ruteplan' # relative to removable media
route_plan_dir = '../route_plans' # relative to 'survey_route_plan.py'
try_unmount = False

rp = route_plan()

# import route plan from removable media to local route plan directory (only required once after media insertion)
route_plan =  rp.import_route_plan_from_removable_media (file_name, import_dir, route_plan_dir, try_unmount)
if rp.error():
	print 'Error:', rp.error_text()
else:
	print 'Route plan imported from removable media'

# load route plan from route plan directory
route_plan = rp.load_route_plan (file_name, route_plan_dir, ';')
if rp.error():
	print 'Error:', rp.error_text()
else:
	print route_plan

