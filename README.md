# HdMap Drawer

## How to test and run?

data preparation:

we will deliver a json file or a attribute based obj (recommended)

```
json like: maps/G3v2.json
```

will be input into the follow class obj:

```

map_tool/lib/map_draw.py:

map = map()


"""
Input the attribute based obj into the load
"""
map.load_new_obj(traf_map)

"""
Input a json str file into the load
check json convert ability
"""
map.load_json()

finally

map.draw_everything()
plt.savefig("./res.pdf")

view the res, containing all the elements, will be fine.

```