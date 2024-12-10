from gymnasium.envs.registration import register

register(
    id="CartPole-v99",
    entry_point="cartpole:CartPoleEnv",
    max_episode_steps=200,
)
