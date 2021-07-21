from gym.envs.registration import register

register(
    id='op3-v0',
    entry_point='gym_op3.env:OP3Env'
)
