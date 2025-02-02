import subprocess

cmd = 'source $HOME/Applications/OpenFOAM/OpenFOAM-v2212/etc/bashrc && ./Allrun.pre'
cmd = 'source $HOME/Applications/OpenFOAM/OpenFOAM-v2212/etc/bashrc && cfd/wind_around_buildings_tutorial/./Allrun.pre'

subprocess.run(cmd,shell=True,executable='/bin/bash')

