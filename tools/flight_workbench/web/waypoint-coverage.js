/* Read-only preview of trial_low_coverage.coverage_waypoints / SearchPolicy.
 * Keep the ordered endpoints (including loop connectors); never use survey_xy.
 */
(function(root){
  'use strict';
  function route(site, selection={}){
    const raw=site._low_coverage_original||{};
    const config={coverage_scan_axis:'y',coverage_cross_inset:.6,coverage_lane_spacing:2.6,coverage_loop_inset:1.8,...raw,...site};
    const pattern=selection.coverage_pattern??config.coverage_pattern??'lawnmower';
    const agl=Number(selection.coverage_agl??config.low_agl??1.8);
    const bounds=site.flight_area?.search_bounds??site.coverage_bounds;
    const validBox=b=>Array.isArray(b)&&b.length===4&&b.every(Number.isFinite)&&b[0]<b[1]&&b[2]<b[3];
    if(!validBox(bounds))throw Error('搜索范围无效，无法显示低位路线');
    const axis=config.coverage_scan_axis,inset=config.coverage_cross_inset,spacing=config.coverage_lane_spacing,loopInset=config.coverage_loop_inset;
    if(!['x','y'].includes(axis)||!Number.isFinite(inset)||inset<=0||!Number.isFinite(spacing)||spacing<.2||spacing>3||!Number.isFinite(loopInset)||loopInset<=0||![1.8,1.9].includes(agl)||!['lawnmower','nested_rectangles'].includes(pattern))throw Error('低位路线参数无效');
    const cross=axis==='y'?0:2,along=cross===0?2:0,round=v=>Number(v.toFixed(10));
    const usable=bounds.slice();usable[cross]+=inset;usable[cross+1]-=inset;
    if(!(usable[cross]<usable[cross+1]))throw Error('横向内收后没有可用搜索范围');
    const outer=usable.map(round);let points=[],source;
    if(Object.hasOwn(config,'search_waypoints_xy')){
      points=structuredClone(config.search_waypoints_xy);source='site.search_waypoints_xy';
    }else if(pattern==='nested_rectangles'){
      let rectangles=config.coverage_rectangles;
      if(!Object.hasOwn(config,'coverage_rectangles')){
        const inner=outer.slice();inner[cross]+=loopInset;inner[cross+1]-=loopInset;inner[along]+=loopInset/2;inner[along+1]-=loopInset/2;
        rectangles=[outer,inner.map(round)];
      }
      if(!Array.isArray(rectangles)||rectangles.length!==2||!rectangles.every(validBox))throw Error('回字形内外框无效，请调整范围或内收距离');
      const [o,i]=rectangles;if(!(o[0]<i[0]&&i[0]<i[1]&&i[1]<o[1]&&o[2]<i[2]&&i[2]<i[3]&&i[3]<o[3]))throw Error('回字形内框必须严格位于外框内');
      rectangles.forEach(([x0,x1,y0,y1],index)=>{
        const loop=axis==='y'?[[x0,y0],[x0,y1],[x1,y1],[x1,y0],[x0,y0]]:[[x0,y0],[x1,y0],[x1,y1],[x0,y1],[x0,y0]];
        if(index)points.push(axis==='y'?[loop[0][0],points.at(-1)[1]]:[points.at(-1)[0],loop[0][1]]);
        points.push(...loop);
      });
      source=Object.hasOwn(config,'coverage_rectangles')?'nested_rectangles(coverage_rectangles)':'nested_rectangles(inset_search_bounds)';
    }else{
      if(Math.ceil((outer[cross+1]-outer[cross])/spacing)+1>50)throw Error('低位路线超过100个端点');
      let lane=outer[cross],forward=true;
      while(true){
        const start=forward?outer[along]:outer[along+1],end=forward?outer[along+1]:outer[along];
        points.push(...(axis==='y'?[[lane,start],[lane,end]]:[[start,lane],[end,lane]]));
        if(lane>=outer[cross+1])break;
        lane=Math.min(lane+spacing,outer[cross+1]);forward=!forward;
      }
      source='SearchPolicy(inset_search_bounds)';
    }
    if(!Array.isArray(points)||points.length<2||points.length>100||points.some(p=>!Array.isArray(p)||p.length!==2||!p.every(Number.isFinite)||p[0]<bounds[0]||p[0]>bounds[1]||p[1]<bounds[2]||p[1]>bounds[3]))throw Error('低位路线端点须位于实测搜索范围内（2～100点）');
    return {points,source,pattern,agl,axis,inset,spacing,loopInset};
  }
  root.WaypointCoverage={route};
  if(typeof module!=='undefined')module.exports=root.WaypointCoverage;
})(globalThis);
