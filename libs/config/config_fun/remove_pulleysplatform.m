function remove_pulleysplatform(btn,tg,PointTab,PulleyTab)
if length(PulleyTab.Children)<1
        msgbox('No pulleys left')
else
    delete(PointTab.Children(1))
    delete(PulleyTab.Children(1))
end
end