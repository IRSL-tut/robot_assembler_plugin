# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.16.4
#   kernelspec:
#     display_name: Choreonoid
#     language: python
#     name: choreonoid
# ---

# %%
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())

# %% [markdown]
# # import 他

# %%
import cnoid.RobotAssembler
proj_dir = '/choreonoid_ws/install/share/choreonoid-2.3/robot_assembler/irsl'
dc = DrawCoords(width=5)
di = DrawInterface()

# %% [markdown]
# # Settings.yamlのロード

# %%
ras = cnoid.RobotAssembler.RobotAssemblerSettings()
ras.parseYaml(proj_dir + '/irsl_assembler_config.yaml')

# %% [markdown]
# # パーツ情報の取得

# %%
pt = ras.getParts('tof-sensor')

# %% [markdown]
# # 形状の描画

# %%
vis  = pt.createVisual(proj_dir) ## create visual of parts
di.addObject(vis)
di.viewAll()

# %% [markdown]
# # 基準座標系

# %%
dc.addCoords(coordinates())

# %% [markdown]
# # センサ座標系

# %%
if pt.numberOfExtraData > 0:
    ex = pt.getExtraData(0)
    print('sensor_local: {}'.format(ex.coords))
    dc.clear()
    dc.addCoords(ex.coords)

# %%
# %display
