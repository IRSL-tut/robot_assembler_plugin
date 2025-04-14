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
# # Make robot model using python

# %% [markdown]
# ## Making initial parts

# %%
base = ru.makeRobotFromKey('XL-430')
base_name = base.rootParts().name

# %% [markdown]
# ## Making parts (to be attached)

# %%
pt0 = ru.makeParts('Hinge_Short_Plug')

# %% [markdown]
# ## Attach parts

# %%
base.attach(pt0, pt0.name + '/side_connect_point', base_name + '/horn', 'default')

# %% [markdown]
# ## Making parts2 (to be attached)

# %%
pt1=ru.makeParts('Hinge_Short_Socket')

# %% [markdown]
# ## Attach parts2

# %%
base.attach(pt1, pt1.name + '/socket_3_3', pt0.name + '/plug_3_0', 'default')

# %% [markdown]
# ## Creating cnoid.Body.Body

# %%
cr_body = cra.RoboasmBodyCreator(proj_dir)
bdi = cr_body.createBodyRaw(base)

# %% [markdown]
# # View Body

# %%
itm=ib.BodyItem()
itm.setBody(bdi)
ib.getRootItem().addChildItem(itm)
itm.setChecked(True)
ib.viewAll()

# %%
# %display

# %% [markdown]
# ## Export .body

# %%
iu.exportBody('test.body', bdi)
