from pathlib import Path

import yaml


class Task2Config:

    def __init__(
        self,
        config_directory,
    ):

        self.config_directory = Path(
            config_directory
        )

        self.device = self._load(
            'device.yaml'
        )

        self.communication = self._load(
            'communication.yaml'
        )

        self.task = self._load(
            'task_points.yaml'
        )

        self.safety = self._load(
            'safety.yaml'
        )

        self._validate()

    def _load(
        self,
        filename,
    ):

        path = (
            self.config_directory
            /
            filename
        )

        if not path.exists():

            raise RuntimeError(
                f'Configuration file not found: {path}'
            )

        with path.open(
            'r',
            encoding='utf-8',
        ) as file:

            data = yaml.safe_load(
                file
            )

        if data is None:

            raise RuntimeError(
                f'Configuration file is empty: {path}'
            )

        return data

    def _validate(self):

        mode = int(
            self.device[
                'mode'
            ]
        )

        if mode not in (
            0,
            1,
        ):

            raise RuntimeError(
                'device.yaml mode must be '
                '0 (simulation) or 1 (real robot).'
            )

    @property
    def mode(self):

        return int(
            self.device[
                'mode'
            ]
        )

    @property
    def is_simulation(self):

        return (
            self.mode
            ==
            0
        )

    @property
    def is_real_robot(self):

        return (
            self.mode
            ==
            1
        )

    def mode_name(self):

        if self.is_simulation:

            return 'SIMULATION'

        return 'REAL_ROBOT'
