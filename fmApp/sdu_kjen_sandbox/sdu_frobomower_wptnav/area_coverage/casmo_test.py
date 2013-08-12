from area_coverage_casmo import area_coverage_casmo
#from math import pi



def print_state(state, pos):
    print '%d: %.1f %.1f' % (state, pos[0], pos[1])



# initialize casmo
casmo = area_coverage_casmo()

# set width of the implement (from launch parameter?)
casmo.param_set_width (0.8)

# optionaly set the default length to drive unless the user turns before that
casmo.param_set_default_length (100)


wptlist = []
print 'user pressed up button in auto mode (robot at 0.0 pointing East)'
wptlist.append(casmo.start([0.0, 0.0], 0)) # when switching to auto call start(pos, orientation)
print_state (casmo.state, wptlist[-1])

print 'user pressed left button while at approx (50,1) the ,1 is to illustrate inaccuracy'
wptlist.append(casmo.turn_left ([50.0, 1.0])) # when user presses left call turn_left(pos) 
print_state (casmo.state, wptlist[-1])

print 'robot reached first left turn wpt thus requesting the next.'
wptlist.append(casmo.goto_next ()) # whenever robot has reached a waypoint call goto_next()
print_state (casmo.state, wptlist[-1])

print 'user pressed right button while driving left'
wptlist.append(casmo.turn_right ([0,0.8])) # when user presses right call turn_right (pos)
print_state (casmo.state, wptlist[-1])

print 'robot reached first right turn wpt thus requesting the next'
wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])

wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])

wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])

wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])

wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])

wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])

wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])

print 'Example of the user reseting the length while driving to the left'
wptlist.append(casmo.reset_length())
print_state (casmo.state, wptlist[-1])

print 'Then at (-25,4) the user presses the turn right button'
wptlist.append(casmo.turn_right ([-25,4.0]))
print_state (casmo.state, wptlist[-1])


wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])

wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])

wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])

wptlist.append(casmo.goto_next ())
print_state (casmo.state, wptlist[-1])
