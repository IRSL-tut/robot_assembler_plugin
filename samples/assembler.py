##
## TODO
##
#>-- assembler as Body
#>- add assembler_id(str) to body
#>- locate points associate with the part (as the body)
#>- add link and update points
#>- detach by link
#>- delete a link ( tip link )
#>- delete a link on middle  -> detach by a link and delete the link (as a tip link )
#>- virtual attach
#>- calculate IK
#>
#>** click in python
#>* draw connecting point as clickable
#>* callback function (manager class by python)

#
# update body
#
orgitem = cnoid.BodyPlugin.BodyItem("org")
obd=ru.loadRobot('original.body')
orgitem.setBody(obd)

ib.RootItem.instance.addChildItem(orgitem)
orgitem.setChecked(True)
orgitem.setSelected(True)

newitem = cnoid.BodyPlugin.BodyItem("new")
nbd=ru.loadRobot('new.body')
## convert state obd => nbd
newitem.setBody(nbd)

newitem.replace(orgitem)

