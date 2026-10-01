"""Road-centre routing; Nav2 still validates every physical segment."""
import heapq,math

def road_route(network,start,goal,blocked=()):
    nodes=network['nodes'];edges=network['edges']
    nearest=lambda p:min(nodes,key=lambda k:math.dist(nodes[k],(p['x'],p['y'])))
    source,target=nearest(start),nearest(goal)
    queue=[(0.,source,[])];seen=set();blocked={frozenset(e) for e in blocked}
    while queue:
        cost,key,path=heapq.heappop(queue)
        if key in seen:continue
        seen.add(key);path=path+[key]
        if key==target:return [(k,nodes[k]) for k in path]
        for a,b in edges:
            if frozenset((a,b)) in blocked:continue
            other=b if a==key else a if b==key else None
            if other and other not in seen:heapq.heappush(queue,(cost+math.dist(nodes[key],nodes[other]),other,path))
    raise RuntimeError('Нет свободной ветки дорожной сети')
