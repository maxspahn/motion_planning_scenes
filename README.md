# Motion Planning Scenes

Motion planning consists of finding a path from a state A to a goal state B while avoiding
obstacles.

There is a wide range of motion planning libraries that focus mostly on motion planning problems
formulated in the configuration space [OMPL](https://ompl.kavrakilab.org/). This approach
is usually based on inverse kinematics to transform real-world goals into suitable
configurations. 

This repository formulates a generic motion planning scene, including both moving and
static obstacles and a generic formulation of goals.

## Obstacles

Obstacles can be roughly split into two categories, moving and static.

## Goals

Goals for motion planning should not depend on the robot's structure, neither should they
involve orientation that are generally hard to obtain and hardly human understandable.

## Structure (Beta)

<tr><td colspan=2 align="center">
    <b>Structure</b><br />
    [<a href="http://mermaid-js.github.io/mermaid/#/gantt">docs</a> - <a
href="https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW01vdGlvblBsYW5uaW5nU2NlbmVdXG4gICAgQltNb3Rpb25QbGFubmluZ0dvYWxdXG4gICAgQ1tNb3Rpb25QbGFubmluZ1NjZW5lXVxuICAgIEEgLS0-IEJcbiAgICBBIC0tPiBDXG4gICIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2UsImF1dG9TeW5jIjp0cnVlLCJ1cGRhdGVEaWFncmFtIjpmYWxzZX0)](https://mermaid.live/edit#eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW01vdGlvblBsYW5uaW5nU2NlbmVdXG4gICAgQltNb3Rpb25QbGFubmluZ0dvYWxdXG4gICAgQ1tNb3Rpb25QbGFubmluZ1NjZW5lXVxuICAgIEEgLS0-IEJcbiAgICBBIC0tPiBDXG4gICIsIm1lcm1haWQiOiJ7XG4gIFwidGhlbWVcIjogXCJkZWZhdWx0XCJcbn0iLCJ1cGRhdGVFZGl0b3IiOmZhbHNlLCJhdXRvU3luYyI6dHJ1ZSwidXBkYXRlRGlhZ3JhbSI6ZmFsc2V9">live
editor</a>]
[![]()
</td></tr>
