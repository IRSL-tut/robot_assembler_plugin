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
import cnoid.RobotAssembler as cra
proj_dir = '/choreonoid_ws/install/share/choreonoid-2.3/robot_assembler/irsl'
ru = cra.RoboasmUtil(proj_dir + '/irsl_assembler_config.yaml')

# %% [markdown]
# ## Loading .roboasm

# %%
raf = cra.cnoidRAFile('test.roboasm')
rbt = ru.makeRobot(raf)

# %% [markdown]
# ## Creating a instance of BodyCreator

# %%
cr_body = cra.RoboasmBodyCreator(proj_dir)


# %% [markdown]
# ## Making a instance of Body (with history)

# %%
bd_w_hist = cr_body.createBody(rbt, raf, "HOGE")

# %% [markdown]
# ## Making a instance of Body (without history)

# %%
bdi_wo_hist = cr_body.createBodyRaw(rbt)

# %% [markdown]
# ## View Body

# %%
itm=ib.BodyItem()
itm.setBody(bd_w_hist)
ib.getRootItem().addChildItem(itm)
itm.setChecked(True)
ib.viewAll()

# %%
# %display

# %% [markdown]
# ## Export .body

# %%
iu.exportBody('test.body', bd_w_hist)
