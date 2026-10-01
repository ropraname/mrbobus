#!/usr/bin/env python3
"""Build a navigation prior from fixed field geometry; preserve LiDAR reference.
Movable rubble/vehicles are deliberately not painted into the static layer.
"""
import argparse,json
from pathlib import Path
import numpy as np
import yaml

def inside(x,y,polygon):
    result=np.zeros(x.shape,dtype=bool)
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        if a[1]==b[1]:continue
        result^=((a[1]>y)!=(b[1]>y))&(x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0])
    return result

def build(config):
    r=.025;origin=(-2.35,-2.35);n=192
    x,y=np.meshgrid(origin[0]+(np.arange(n)+.5)*r,origin[1]+(np.arange(n)+.5)*r)
    grid=np.zeros((n,n),np.uint8)  # Outside enclosure stays impassable in scale mode.
    # Measured enclosure extents from map annotation. Walls remain occupied.
    field=(x>-2.056)&(x<2.122)&(y>-1.947)&(y<2.193);grid[field]=254
    wall=field&((x<-2.006)|(x>2.072)|(y<-1.897)|(y>2.143));grid[wall]=0
    objects={o['id']:o for o in config['objects']}
    # Traversable lawn incurs a finite penalty. Paint before buildings so their
    # footprint remains lethal even where it overlaps this 80cm square.
    for area in objects['round_tower'].get('surroundings',[]):
        if area['id']=='round_tower_lawn':
            grid[inside(x,y,area['footprint'])&field&~wall]=140
    for key in ('round_tower','panel_house','park'):
        grid[inside(x,y,objects[key]['footprint'])]=0
    # park is the conservative central water/vegetation island, not a road.
    # Keep bridge decks traversable; retain thin rail boundaries.
    for area in config['special_areas']:
        if area['type']!='bridge':continue
        poly=area['footprint'];xs=[p[0] for p in poly];ys=[p[1] for p in poly]
        deck=inside(x,y,poly);grid[deck]=254
        if max(xs)-min(xs)>max(ys)-min(ys):rails=deck&((y<min(ys)+.025)|(y>max(ys)-.025))
        else:rails=deck&((x<min(xs)+.025)|(x>max(xs)-.025))
        grid[rails]=0
    return np.flipud(grid),{'image':'navigation.pgm','mode':'scale','resolution':r,'origin':[*origin,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('semantic');p.add_argument('output');a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    grid,meta=build(yaml.safe_load(Path(a.semantic).read_text()))
    (out/'navigation.pgm').write_bytes(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode()+grid.tobytes())
    (out/'navigation.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
