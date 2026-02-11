# PD controller parameters (set these beforehand)
Kp = 2.0
Kd = 0.5
r = 10.0      # reference
dt = 1.0     # sampling time

upper_bound = 100.0
lower_bound = 0.0

# internal state
_e_prev = 0.0
_initialized = False

def pd_control(x):
    global _e_prev, _initialized

    e = r - x

    if not _initialized:
        de = 0.0
        _initialized = True
    else:
        de = (e - _e_prev) / dt

    u = Kp * e + Kd * de

    if u >= upper_bound:
        u = upper_bound
    
    if u <= lower_bound:
        u = lower_bound

    _e_prev = e
    return u


# Example
if __name__ == "__main__":
    x = 0.2
    u = pd_control(x)
    print("u =", u)
