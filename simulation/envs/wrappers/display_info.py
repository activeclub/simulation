"""A collections of rendering-based wrappers.

* ``RenderCollection`` - Collects rendered frames into a list
* ``RecordVideo`` - Records a video of the environments
* ``HumanRendering`` - Provides human rendering of environments with ``"rgb_array"``
"""

from __future__ import annotations

from typing import Any, SupportsFloat

import gymnasium as gym
from gymnasium.core import ActType, ObsType
from gymnasium.error import DependencyNotInstalled
import numpy as np


class DisplayInfo(
    gym.Wrapper[ObsType, ActType, ObsType, ActType], gym.utils.RecordConstructorArgs
):
    """Collect rendered frames of an environment such ``render`` returns a ``list[RenderedFrame]``.

    No vector version of the wrapper exists.

    Example:
        Return the list of frames for the number of steps ``render`` wasn't called.
        >>> import gymnasium as gym
        >>> env = gym.make("LunarLander-v3", render_mode="rgb_array")
        >>> env = RenderCollection(env)
        >>> _ = env.reset(seed=123)
        >>> for _ in range(5):
        ...     _ = env.step(env.action_space.sample())
        ...
        >>> frames = env.render()
        >>> len(frames)
        6

        >>> frames = env.render()
        >>> len(frames)
        0

        Return the list of frames for the number of steps the episode was running.
        >>> import gymnasium as gym
        >>> env = gym.make("LunarLander-v3", render_mode="rgb_array")
        >>> env = RenderCollection(env, pop_frames=False)
        >>> _ = env.reset(seed=123)
        >>> for _ in range(5):
        ...     _ = env.step(env.action_space.sample())
        ...
        >>> frames = env.render()
        >>> len(frames)
        6

        >>> frames = env.render()
        >>> len(frames)
        6

        Collect all frames for all episodes, without clearing them when render is called
        >>> import gymnasium as gym
        >>> env = gym.make("LunarLander-v3", render_mode="rgb_array")
        >>> env = RenderCollection(env, pop_frames=False, reset_clean=False)
        >>> _ = env.reset(seed=123)
        >>> for _ in range(5):
        ...     _ = env.step(env.action_space.sample())
        ...
        >>> _ = env.reset(seed=123)
        >>> for _ in range(5):
        ...     _ = env.step(env.action_space.sample())
        ...
        >>> frames = env.render()
        >>> len(frames)
        12

        >>> frames = env.render()
        >>> len(frames)
        12

    Change logs:
     * v0.26.2 - Initially added
    """

    def __init__(
        self,
        env: gym.Env[ObsType, ActType],
    ):
        """Initialize a :class:`RenderCollection` instance.

        Args:
            env: The environment that is being wrapped
            pop_frames (bool): If true, clear the collection frames after ``meth:render`` is called. Default value is ``True``.
            reset_clean (bool): If true, clear the collection frames when ``meth:reset`` is called. Default value is ``True``.
        """
        gym.utils.RecordConstructorArgs.__init__(self)
        gym.Wrapper.__init__(self, env)

        self.episode_id = -1

    def step(
        self, action: ActType
    ) -> tuple[ObsType, SupportsFloat, bool, bool, dict[str, Any]]:
        """Perform a step in the base environment and collect a frame."""
        output = super().step(action)
        self._render_frame()
        return output

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[ObsType, dict[str, Any]]:
        """Reset the base environment, eventually clear the frame_list, and collect a frame."""
        obs, info = super().reset(seed=seed, options=options)
        self.episode_id += 1

        return obs, info

    def render(self):
        return self._render_frame()

    def _render_frame(self):
        try:
            import pygame
        except ImportError as e:
            raise DependencyNotInstalled(
                'pygame is not installed, run `pip install "gymnasium[classic-control]"`'
            ) from e

        assert self.env.render_mode is not None

        last_rgb_array = self.env.render()

        assert isinstance(
            last_rgb_array, np.ndarray
        ), f"Expected `env.render()` to return a numpy array, actually returned {type(last_rgb_array)}"

        rgb_array = np.transpose(last_rgb_array, axes=(1, 0, 2))

        surf = pygame.surfarray.make_surface(rgb_array)

        text_font = pygame.font.SysFont("Arial", 36)
        img = text_font.render(f"episode: {self.episode_id}", True, (0, 0, 0))
        surf.blit(img, (50, 50))

        return np.transpose(np.array(pygame.surfarray.pixels3d(surf)), axes=(1, 0, 2))
