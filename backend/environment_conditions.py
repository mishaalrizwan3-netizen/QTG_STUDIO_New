"""Apply operator-selected Test Settings weather and centre of gravity."""


def weather_snapshot(cfg):
    return {'altitude_msl': cfg['alt_ft'], 'oat': cfg['temperature_c'],
            'wind_spd': cfg.get('wind_speed'), 'wind_dir': cfg.get('wind_direction'),
            'atmospheric_pressure_hpa': cfg.get('atmospheric_pressure_hpa')}


def apply_environment(sim, cfg):
    from interfaces.brunner_base.src.replay_support import reference_weather_metar
    metar = reference_weather_metar(weather_snapshot(cfg), sim.get_global_weather_metar())
    if not metar or not sim.set_global_weather_metar(metar):
        raise RuntimeError('Selected wind, temperature or atmospheric pressure was rejected')
    if 'cg_percent' in cfg:
        setter = getattr(sim, 'set_initial_cg_percent', None)
        if setter is None or not setter(cfg['cg_percent']):
            raise RuntimeError('Selected CG was rejected; rebuild the P3D bridge with CG initialization support')
