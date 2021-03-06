import subprocess
import yaml

def ticcmd(*args):
    return subprocess.check_output(['ticcmd'] + list(args))

status = yaml.load(ticcmd('-s', '--full'))
position = status['Current position']
print("Current position is {}.".format(position))
new_target = -42500#-200 if position > 0 else 200
print("Setting target position to {}.".format(new_target))
while True:
    new_target-=500
    ticcmd('--exit-safe-start', '--position', str(new_target))
    print(status['Forward limit active'],status['Reverse limit active'])