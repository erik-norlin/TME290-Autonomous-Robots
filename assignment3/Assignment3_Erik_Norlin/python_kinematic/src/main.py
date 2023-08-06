from predict import *
from estimate import *
from update import *


freq = 10
dt = 1 / freq
t = 0
R = 0.27
test = 0

predict = Predict(dt, R)
update = Update(dt)
estimate = Estimate()


while True:
    print('t={:.1f}'.format(t))

    pred = predict.predict(estimate, t)
    est = update.update(pred, estimate, t, test)
    estimate.set_est(est)

    t += dt
    if t > 40:
        estimate.save()
        exit()