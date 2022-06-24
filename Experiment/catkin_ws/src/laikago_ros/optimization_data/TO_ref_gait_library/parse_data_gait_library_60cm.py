import scipy.io
import numpy as np

print("script run1")
data = scipy.io.loadmat("jumping_60cm_stepping_stone_noshaking_interp_cs10202540")
print("script run2")
data = {k:v for k, v in data.items() if k[0] != '_'}
print("script run3")
parameter = data.keys()
print("script run4")
for i in parameter:
	print(i)
	if(i == 'tau'):
		np.savetxt(("60cm_tau.csv".format(i)), data.get(i), delimiter=",")
		print("done taus")
	if(i == 'Q'):
		np.savetxt(("60cm_Q.csv".format(i)), data.get(i), delimiter=",")
		print("done Q")
	if(i == 'Ff'):
		np.savetxt(("60cm_Ff.csv".format(i)), data.get(i), delimiter=",")
		print("done Ff")
	if(i == 'Fr'):
		np.savetxt(("60cm_Fr.csv".format(i)), data.get(i), delimiter=",")
		print("done Fr")

print("script run5")
