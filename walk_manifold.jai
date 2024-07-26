walk_manifold_move_by :: (manager : *Entity_Manager, mover : Mover_Representation, starting_position : Vector3, requested_delta : Vector2, walkable_entities : []*Entity, blockers : []*Entity, cell_dim := 0.125, cell_count := V2s.{13, 13}) -> nearest_valid_position : Vector3 {
    profile_function();

    survey := create_survey(manager, mover, starting_position, walkable_entities, blockers, cell_dim, cell_count,,allocator=temp);
    generate_survey(manager, survey,, allocator=temp);

    diagram_phase_begin(*wm_diagram, "Find best Starting Ring");
    diagram_colour(*wm_diagram, V4_CYAN);

    best_starting_ring : *Ring;
    best_starting_ring_d2 := FLOAT32_MAX;

    for *r : survey.rings {
        p, d2 := find_closest_point_in_ring(r, starting_position);
        diagram_add_point(*wm_diagram, p);
        if d2 < best_starting_ring_d2 {
            best_starting_ring    = r;
            best_starting_ring_d2 = d2;
        }
    }

    desired_position := starting_position;
    desired_position.xy += requested_delta;

    if !best_starting_ring {
        log("WARNING[walk_manifold_move_by]: No valid rings found. Allowing any movement.\n");
        return desired_position;
    }

    surface : Walkable_Surface;
    surface.rings.allocator = temp;
    add_rings_to_surface(survey, *surface, best_starting_ring);


    diagram_phase_begin(*wm_diagram, "Find best ending ring");
    diagram_colour(*wm_diagram, V4_WHITE);

    best_ending_position, best_ring, best_ring_d2 := find_closest_point(*surface, desired_position);

    diagram_colour(*wm_diagram, V4_GREEN);
    diagram_add_point(*wm_diagram, best_ending_position);

    return best_ending_position;
}


create_survey :: (manager : *Entity_Manager, mover_ : Mover_Representation, starting_position : Vector3, walkable_entities : []*Entity, blockers : []*Entity, cell_dimension := 0.125, cell_count := V2s.{13, 13}) -> *Walk_Manifold_Survey {
    profile_function();
    using survey := New(Walk_Manifold_Survey);

    survey.cell_dim = cell_dimension;
    survey.inv_cell_dim = 1.0 / survey.cell_dim;

    survey.mover = mover_;

    starting_base_tile_position : Vector2;
    starting_base_tile_position.x = cast(s32)(starting_position.x * inv_cell_dim) * cell_dim;
    starting_base_tile_position.y = cast(s32)(starting_position.y * inv_cell_dim) * cell_dim;

    survey.tile_count = cell_count; 
    evaluation_dims := Vector2.{xx tile_count.x * cell_dim, xx tile_count.y * cell_dim};
    evaluation_radius := length(evaluation_dims) * 0.5;

    survey.aabb.min.xy = starting_base_tile_position - evaluation_dims*0.5;
    survey.aabb.max.xy = survey.aabb.min.xy + evaluation_dims;
    survey.aabb.min.z = starting_position.z - 100.0;
    survey.aabb.max.z = starting_position.z + survey.mover.height * 1.5;

    diagram_phase_begin(*wm_diagram, "Define Bounds");
    diagram_colour(*wm_diagram, V4_GREEN);
    diagram_thickness(*wm_diagram, 0.1);
    diagram_add_box(*wm_diagram, survey.aabb);

    normal_vertical_threshold := cos(to_radians(65.0));

    //@Speed: probably want get_entities_in_AABB and we may be able
    //        to just reuse the same spatial query for both walkables
    //        and blockers

    diagram_phase_begin(*wm_diagram, "Collect Walkable Triangles");
    diagram_colour(*wm_diagram, V4_GREEN);
    diagram_thickness(*wm_diagram, 0.05);
    diagram_add_box(*wm_diagram, survey.aabb);

    for entity : walkable_entities {
        profile_block("create_survey gather walkables");
        triangles := get_triangles(entity);

        for tri : triangles {
            e12 := tri.p2 - tri.p1;
            e02 := tri.p2 - tri.p0;

            normal := cross(e02, e12);//Not yet normalized

            twice_horz_area := normal.z;

            if twice_horz_area <= 0.00001 {
                continue;
            };

            normal_length := length(normal);
            normal.z /= normal_length;

            if normal.z <= normal_vertical_threshold { continue; };

            normal.xy /= normal_length;

            if !collide(survey.aabb, tri, normal) { continue; }
            diagram_colour(*wm_diagram, V4_GREEN);

            walkable_tri := array_add(*survey.walkable_triangles);
            walkable_tri.tri = tri;

            walkable_tri.e12 = e12.xy;
            walkable_tri.e02 = e02.xy; 

            walkable_tri.min_pole_index, walkable_tri.max_pole_index = get_pole_index_range(survey, tri.p0.xy, tri.p1.xy, tri.p2.xy, 0.1);

            walkable_tri.bounds = rect_from_bounds(tri.p0.xy, tri.p1.xy, tri.p2.xy);

            walkable_tri.inv_twice_horz_area = 1.0 / twice_horz_area;
            diagram_add_triangle(*wm_diagram, tri);
        }
    }

    {
        edge_cache.allocator = context.allocator;

        //@Speed: Collapse all of this stuff into a single allocation.
        edge_cache.last_x_edges[0].vertices = NewArray(walkable_triangles.count, Cached_Vertex);
        edge_cache.last_x_edges[1].vertices = NewArray(walkable_triangles.count, Cached_Vertex);

        edge_cache.last_row_edges[0] = NewArray(tile_count.x, Cell_Side_Cache);
        edge_cache.last_row_edges[1] = NewArray(tile_count.x, Cell_Side_Cache);
        for *row : edge_cache.last_row_edges[0] {
            row.vertices = NewArray(walkable_triangles.count, Cached_Vertex);
        }
        for *row : edge_cache.last_row_edges[1] {
            row.vertices = NewArray(walkable_triangles.count, Cached_Vertex);
        }

    }

    diagram_phase_begin(*wm_diagram, "Collect Blockers");
    diagram_colour(*wm_diagram, V4_GREEN);
    diagram_thickness(*wm_diagram, 0.1);
    diagram_add_box(*wm_diagram, survey.aabb);

    for entity : blockers {
        profile_block("create_survey gather blockers");

        if entity.type == {
            case Terrain;
                tris := get_triangles(entity);
                for t : tris {
                    if !collide(survey.aabb, t) continue;
                    add_blocking_tri(survey, t);
                }
        }

        for cv : entity.mesh.collision_volumes {
            if cv.type == {
                case .Cylinder;
                    min_z := entity.position.z + cv.offset.z * entity.scale;
                    max_z := entity.position.z + cv.offset.z * entity.scale;
                    min_z += cv.cylinder.bottom * entity.scale;
                    max_z += cv.cylinder.top    * entity.scale;

                    r := cv.cylinder.radius*entity.scale + survey.mover.radius;

                    p := entity.position.xy + cv.offset.xy * entity.scale;
                    add_blocking_edge(survey, p, p, min_z, max_z, r); 
                case .Convex_Hull;
                    //@Speed: test against the region aabb.
                    hull : Blocking_Hull;

                    hull.points = get_hull_points(*cv.convex_hull, entity.transform);
                    bbox := get_bounding_box(hull.points);

                    if !collide(bbox, survey.aabb) continue;
                    
                    min_i, max_i := get_pole_index_range(survey, bbox.min.xy, bbox.max.xy, radius=survey.mover.radius);
                    hull.min_pole_index = min_i;
                    hull.max_pole_index = max_i;
                    hull.bounds.bl = bbox.min.xy;
                    hull.bounds.wh = bbox.max.xy - bbox.min.xy;
                    hull.min_remove_z = bbox.min.z - survey.mover.height;
                    hull.max_remove_z = bbox.max.z - survey.mover.leg_clearance;

                    array_add(*survey.blocking_hulls, hull);
            }
        }
    }


    return survey;
}

get_surfaces :: (survey : *Walk_Manifold_Survey) -> [..]Walkable_Surface {
    surfaces : [..]Walkable_Surface;
    for *r : survey.rings {
        if r.marked continue;
        surface := array_add(*surfaces);
        add_rings_to_surface(survey, surface, r);
    }
    return surfaces;
}


generate_survey :: (manager : *Entity_Manager, using survey : *Walk_Manifold_Survey) {
    profile_function();
    {
        //init pole info

        survey.poles = NewArray((survey.tile_count.x+1)*(survey.tile_count.y+1), Pole);

        for y_index : 0..tile_count.y {
            for x_index : 0..tile_count.x {
                pole := *survey.poles[x_index + y_index*(tile_count.x+1)];
                pole.world_position = survey.aabb.min.xy;
                pole.world_position.x += x_index*survey.cell_dim;
                pole.world_position.y += y_index*survey.cell_dim;
            }
        }
    }

    rasterize_triangles_into_poles(survey);


    diagram_phase_begin(*wm_diagram, "Passing Points");
    diagram_colour(*wm_diagram, V4_GREEN);
    for pole : poles {
        for z_range : pole.z_ranges {
            diagram_add_point(*wm_diagram, v3(pole.world_position, z_range.max));
        }
    }


    x_cache := Cache_Pair.{*edge_cache.last_x_edges[0], *edge_cache.last_x_edges[1]};

    write_row_cache : *[]Cell_Side_Cache;
    read\_row_cache : *[]Cell_Side_Cache;
    write_row_cache = *edge_cache.last_row_edges[0];
    read\_row_cache = *edge_cache.last_row_edges[1];

    //We take a view of the array before adding the eroded edges to it, since we don't want to use those
    // edges while finding the others.
    survey.active_blocking_edges = survey.blocking_edges;

    erode_walkable_surfaces := true;
    if erode_walkable_surfaces {
        profile_block("Find erosion edges");
        diagram_phase_begin(*wm_diagram, "Erosion Edges");
        diagram_colour(*wm_diagram, V4_RED);
        diagram_thickness(*wm_diagram, 0.02);
        for y : 0..tile_count.y-1 {
            reset(x_cache.read\_cache);
            reset(x_cache.write_cache);
            for x : 0..tile_count.x-1 {
                poles : [4]*Pole;
                //We do them in a loop order, not a zig-zag
                poles[0] = *survey.poles[x+0 + (y+0)*(tile_count.x+1)];
                poles[1] = *survey.poles[x+1 + (y+0)*(tile_count.x+1)];
                poles[2] = *survey.poles[x+1 + (y+1)*(tile_count.x+1)];
                poles[3] = *survey.poles[x+0 + (y+1)*(tile_count.x+1)];

                xc := x_cache;
                if x == 0 then xc.read_cache = null;

                y_cache := Cache_Pair.{*write_row_cache.*[x], *read_row_cache.*[x]};
                //@Speed: we could also not set the write cache if we are on the last row.
                if y == 0 then y_cache.read_cache = null;

                compute_rings(survey, poles, xc, y_cache, for_edges=true);

                //assert(x_cache.read_cache.read_count == x_cache.read_cache.write_count);
                reset(x_cache.read_cache );
                Swap(*x_cache.write_cache, *x_cache.read_cache);
            }
            reset(read_row_cache);
            Swap(*write_row_cache, *read_row_cache);
        }
    }

    {//Reset edge cache
        using edge_cache;
        reset(*last_x_edges[0]);
        reset(*last_x_edges[1]);
        reset(*last_row_edges[0]);
        reset(*last_row_edges[1]);
    }


    diagram_phase_begin(*wm_diagram, "rasterize_blocking_edges");
    survey.active_blocking_edges = survey.blocking_edges;
    rasterize_blocking_edges(survey);
    diagram_phase_begin(*wm_diagram, "rasterize_blocking_hulls");
    rasterize_blocking_hulls(survey);
    rasterize_blocking_tris(survey);



    //Re-Sort, since we may have done unordered array removals
    for pole : poles {
        quick_sort(pole.z_ranges, (a) => -a.max);
    }

    diagram_phase_begin(*wm_diagram, "Compute Rings");
    diagram_colour(*wm_diagram, V4_WHITE);
    diagram_thickness(*wm_diagram, 0.01);

    rings_y_cache := NewArray(survey.tile_count.x, V2s,,allocator=temp);

    for y : 0..tile_count.y-1 {
        profile_block("Find final rings");
        reset(x_cache.read\_cache);
        reset(x_cache.write_cache);

        last_ring_index_span : V2s;
        for x : 0..tile_count.x-1 {
            poles : [4]*Pole;
            //We do them in a loop order, not a zig-zag
            poles[0] = *survey.poles[x+0 + (y+0)*(tile_count.x+1)];
            poles[1] = *survey.poles[x+1 + (y+0)*(tile_count.x+1)];
            poles[2] = *survey.poles[x+1 + (y+1)*(tile_count.x+1)];
            poles[3] = *survey.poles[x+0 + (y+1)*(tile_count.x+1)];

            xc := x_cache;
            if x == 0 then xc.read_cache = null;

            y_cache := Cache_Pair.{*write_row_cache.*[x], *read_row_cache.*[x]};
            if y == 0 then y_cache.read_cache = null;

            ring_index_span := compute_rings(survey, poles, xc, y_cache, for_edges=false);


            rings := get_rings(survey, ring_index_span);
            #if ENABLE_DIAGRAMS {
                for r : rings {
                    diagram_add_ring(*wm_diagram, r);
                }
            }

            if x > 0 map_edges(survey, last_ring_index_span, ring_index_span);
            last_ring_index_span = ring_index_span;

            if y > 0 then map_edges(survey, rings_y_cache[x], ring_index_span);
            rings_y_cache[x] = ring_index_span;

            //assert(x_cache.read_cache.read_count == x_cache.read_cache.write_count);
            reset(x_cache.read_cache);
            Swap(*x_cache.write_cache, *x_cache.read_cache);
        }
        reset(read\_row_cache);
        Swap(*write_row_cache, *read_row_cache);
    }
}

find_closest_point :: (surface : *Walkable_Surface, p : Vector3) -> Vector3, ring : *Ring, dist_squared : float {
    best_ring     : *Ring;
    best_ring_d2  := FLOAT32_MAX;
    best_position : Vector3 = p;
    for *r : surface.rings {
        p, d2 := find_closest_point_in_ring(r, p);
        diagram_add_point(*wm_diagram, p);

        if d2 < best_ring_d2 {
            best_ring_d2  = d2;
            best_ring     = r;
            best_position = p;
        }
    }
    return best_position, best_ring, best_ring_d2;
}

Mover_Representation :: struct {
    radius : float = 0.25; 
    height : float = 1.8;
    leg_clearance : float = 0.7;
    max_cell_height_change : float = 0.5;
}


Walk_Manifold_Survey :: struct {
    cell_dim : float = 0.125; 
    inv_cell_dim : float;

    walkable_triangles : [..]Walkable_Triangle;
    blocking_edges : [..]Blocking_Edge;
    blocking_hulls : [..]Blocking_Hull;
    blocking_tris  : [..]Blocking_Triangle;

    active_blocking_edges : []Blocking_Edge;

    tile_count : V2s;

    mover : Mover_Representation;

    poles : []Pole;

    rings : [..]Ring;

    aabb : Rect3;

    is_complete := false;

    edge_cache : Edge_Cache;
}

Walkable_Surface :: struct {
    rings : [..]Ring;
}

#scope_file


Edge_Cache :: struct {
    allocator : Allocator;
    //We double buffer so that we can use one cache while creating the other
    last_x_edges   : [2]Cell_Side_Cache;
    last_row_edges : [2][]Cell_Side_Cache;

}

Cached_Vertex :: struct {
    v : Vector3;
    in_pole_z : float;
    found : bool;
}

Cell_Side_Cache :: struct {
    write_count : u16;
    read\_count : u16;

    vertices : []Cached_Vertex;
}

Cache_Pair :: struct {
    write_cache : *Cell_Side_Cache;
    read\_cache : *Cell_Side_Cache;
}

reset :: (using cache : *Cell_Side_Cache) #expand {
    cache.write_count = 0;
    cache. read_count = 0;
}

reset :: (row : *[]Cell_Side_Cache) #expand {
    for *row.* {
        reset(it);
    }
}

Pole :: struct {
    world_position : Vector2;
    z_ranges : [..]Z_Range;
    walkable_index_count : u16;
    blocking_edge\_count : u16;
    blocking_tri\ _count : u16;
}

Z_Range :: struct {
    min : float;
    max : float;
}

Ring :: struct {
    edge_count : s32;
    edges : [6]Ring_Edge;
    
    marked : bool;//Used for graph traversal
}

Ring_Edge :: struct {
    position : Vector3;
    to : s32 = -1;//index into survey.rings
}

Walkable_Triangle :: struct {
    using tri : Triangle;

    e12 : Vector2;
    e02 : Vector2;

    bounds : Rect2;

    inv_twice_horz_area : float;

    min_pole_index : V2s;
    max_pole_index : V2s;
}

Blocking_Edge :: struct {
    p0 : Vector2;
    p1 : Vector2;

    e0_delta : Vector2;
    e0_len_squared : float;
    inv_e0_len_squared : float;

    radius_squared : float;

    expanded_bounds : Rect2;

    min_pole_index : V2s;
    max_pole_index : V2s;

    min_remove_z : float;
    max_remove_z : float;
}

Blocking_Triangle :: struct {
    ps : Vector3_8;

    bounds : Rect2;

    min_pole_index : V2s;
    max_pole_index : V2s;

    min_remove_z : float;
    max_remove_z : float;
}

Blocking_Hull :: struct {
    points : []Vector3_8;
    min_pole_index : V2s;
    max_pole_index : V2s;
    bounds : Rect2;

    min_remove_z : float;
    max_remove_z : float;
}



rasterize_triangles_into_poles :: (using survey : *Walk_Manifold_Survey) {
    profile_function();
    diagram_phase_begin(*wm_diagram, "rasterize_triangles_into_poles");
    for *tri : walkable_triangles {
        for y : tri.min_pole_index.y..tri.max_pole_index.y {
            for x : tri.min_pole_index.x..tri.max_pole_index.x {
                pole := *survey.poles[x + y*(survey.tile_count.x+1)];
                hits, height := evaluate_triangle(tri, pole.world_position);
                if hits {
                    diagram_colour(*wm_diagram, V4_ORANGE);
                    diagram_add_point(*wm_diagram, v3(pole.world_position, height));
                    insert_z_range(*pole.z_ranges, height, mover.height);
                } 
            }
        }
    }
}

compute_rings :: (survey : *Walk_Manifold_Survey, poles : [4]*Pole, x_cache : Cache_Pair, y_cache : Cache_Pair, $for_edges := false) -> ring_index_span : V2s {
    profile_function();
    used_z_count : [4]u32;

    starting_ring_count := survey.rings.count;

    while true {
        valid : [4]bool;
        heights : [4]float;
        highest_index := -1; //@Speed: Could just be a u8
        for poles {
            if used_z_count[it_index] < it.z_ranges.count {
                valid[it_index] = true;
                heights[it_index] = it.z_ranges[used_z_count[it_index]].max;
                highest_index = it_index;//Initialize to known good value
            }
        }

        //There were no hits on these poles
        if highest_index == -1 break;

        highest_z := heights[highest_index];
        for poles {
            if it_index == highest_index continue;
            if !valid[it_index] continue;
            z := heights[it_index]; 
            if z > highest_z {
                highest_index = it_index;
                highest_z = z;
            }
        }

        next_index     := (highest_index+1) & 3;
        opposite_index := (highest_index+2) & 3;
        last_index     := (highest_index+3) & 3;
        highest  := poles[highest_index ];
        next     := poles[next_index    ];
        opposite := poles[opposite_index];
        last     := poles[last_index    ];

        ROOT_2 :: #run sqrt(2.0);

        maximum_z_delta := survey.mover.max_cell_height_change;

        //Don't use points which are too low compared to the starting ring  
        if highest_z - heights[next_index    ] > maximum_z_delta        valid[next_index    ] = false;
        if highest_z - heights[last_index    ] > maximum_z_delta*ROOT_2 valid[last_index    ] = false;
        if highest_z - heights[opposite_index] > maximum_z_delta        valid[opposite_index] = false;

        used_z_count[highest_index] += 1;

        #if for_edges {
            diagram_colour(*wm_diagram, V4_RED);
        } else {
            diagram_colour(*wm_diagram, V4_WHITE);
        }

        Direction :: enum u8 {
            NegX;
            NegY;
            PosX;
            PosY;
        }
        get_side :: (a : int, b : int) -> Direction {
            low := min(a, b);
            hi  := max(a, b);

            if low == {
                case 0;
                    if hi == 3 
                        return .NegX;
                    else
                        return .NegY;
                case 1; return .PosX;
                case 2; return .PosY;
            }
            return .PosX;
        }

        get_caches_for_side :: (side : Direction) -> Cache_Pair #expand {
            pair : Cache_Pair;
            if side == {
                case .NegX; pair.read_cache  = x_cache.read_cache;
                case .NegY; pair.read_cache  = y_cache.read_cache;
                case .PosX; pair.write_cache = x_cache.write_cache;
                case .PosY; pair.write_cache = y_cache.write_cache;
            }
            return pair;
        }

        add_vertex :: inline (r : *Ring, p : Vector3) {
            assert(r.edge_count < 6);
            r.edges[r.edge_count].position = p;
            r.edge_count += 1;
        }

        diagram_colour(*wm_diagram, V4_RED);

        if !valid[next_index] && !valid[opposite_index] && !valid[last_index] {
            p0 := v3(highest.world_position, highest_z);

            p1_side := get_side(highest_index, next_index);
            p2_side := get_side(highest_index, last_index);

            p1_cache := get_caches_for_side(p1_side);
            p2_cache := get_caches_for_side(p2_side);

            p1, p1_found := find_edge(survey, highest, next, highest_z, p1_cache);
            p2, p2_found := find_edge(survey, highest, last, highest_z, p2_cache);
            if !p1_found && !p2_found continue;//Zero-Area ring, ignore it.

            #if for_edges {
                if !p1_found then p1 = p0;
                if !p2_found then p2 = p0;
                min_z := min(p1.z, p2.z);
                max_z := max(p1.z, p2.z);
                diagram_colour(*wm_diagram, V4_RED);
                add_blocking_edge(survey, p1.xy, p2.xy, min_z, max_z + survey.mover.height, survey.mover.radius);
            } else if p1_found && p2_found {
                ring := array_add(*survey.rings);
                ring.edge_count = 3;
                ring.edges[0] = .{p0, -1};
                ring.edges[1] = .{p1, -1};
                ring.edges[2] = .{p2, -1};
            }
        } else if !valid[next_index] && valid[opposite_index] && !valid[last_index] {
            ring : Ring;
            add_vertex(*ring, v3(highest.world_position, highest_z));

            p1_side := get_side(highest_index , next_index);
            p2_side := get_side(opposite_index, next_index);
            p4_side := get_side(opposite_index, next_index);
            p5_side := get_side(highest_index , last_index);

            p1_cache := get_caches_for_side(p1_side);
            p2_cache := get_caches_for_side(p2_side);
            p4_cache := get_caches_for_side(p4_side);
            p5_cache := get_caches_for_side(p5_side);

            p1, p1_found := find_edge(survey, highest , next, highest_z              , p1_cache);
            p2, p2_found := find_edge(survey, opposite, next, heights[opposite_index], p2_cache);
            p4, p4_found := find_edge(survey, opposite, last, heights[opposite_index], p4_cache);
            p5, p5_found := find_edge(survey, highest , last, highest_z              , p5_cache);

            if p1_found then add_vertex(*ring, p1);
            if p2_found then add_vertex(*ring, p2);

            p3 := v3(opposite.world_position, heights[opposite_index]);
            add_vertex(*ring, p3);

            if p4_found then add_vertex(*ring, p4);
            if p4_found then add_vertex(*ring, p5);

            if ring.edge_count == 2 continue;

            used_z_count[opposite_index] += 1;

            #if for_edges {
                edge_i0 := 0;
                if p1_found then edge_i0 += 1;
                edge_i1 := edge_i0 + 1;

                ep0 := ring.edges[edge_i0].position;
                ep1 := ring.edges[edge_i1].position;
                min_z_01 := min(ep0.z, ep1.z);
                max_z_01 := max(ep0.z, ep1.z);

                edge_i3 := 0;
                if p5_found edge_i3 = ring.edge_count-1;
                edge_i2 := (edge_i3 - 1) % ring.edge_count;
                ep2 := ring.edges[edge_i2].position;
                ep3 := ring.edges[edge_i3].position;

                min_z_23 := min(ep2.z, ep3.z);
                max_z_23 := max(ep2.z, ep3.z);

                diagram_colour(*wm_diagram, V4_RED);
                add_blocking_edge(survey, ep0.xy, ep1.xy, min_z_01, max_z_01 + survey.mover.height, survey.mover.radius);
                add_blocking_edge(survey, ep2.xy, ep3.xy, min_z_23, max_z_23 + survey.mover.height, survey.mover.radius);
            } else {
                assert(ring.edge_count >= 3 && ring.edge_count <= 6);
                array_add(*survey.rings, ring);
            }
        } else if !valid[next_index] && !valid[opposite_index] && valid[last_index] {
            ring : Ring;

            p0 := v3(highest.world_position, highest_z);
            add_vertex(*ring, p0); 

            p1_side := get_side(highest_index, next_index    );
            p2_side := get_side(last_index   , opposite_index);
            p1_cache := get_caches_for_side(p1_side);
            p2_cache := get_caches_for_side(p2_side);

            p1, p1_found := find_edge(survey, highest, next    , highest_z          , p1_cache);
            p2, p2_found := find_edge(survey, last   , opposite, heights[last_index], p2_cache);

            if p1_found then add_vertex(*ring, p1);
            if p2_found then add_vertex(*ring, p2);

            p3 := v3(last.world_position, heights[last_index]);
            add_vertex(*ring, p3);

            used_z_count[last_index] += 1;

            #if for_edges {
                if !p1_found then p1 = p0;
                if !p2_found then p2 = p3;

                min_z_12 := min(p1.z, p2.z);
                max_z_12 := max(p1.z, p2.z);
                diagram_colour(*wm_diagram, V4_RED);
                add_blocking_edge(survey, p1.xy, p2.xy, min_z_12, max_z_12 + survey.mover.height, survey.mover.radius);
            } else {
                if ring.edge_count < 3 continue;
                assert(ring.edge_count >= 3 && ring.edge_count <= 6);
                array_add(*survey.rings, ring);
            }
        } else if valid[next_index] && !valid[opposite_index] && !valid[last_index] {
            ring : Ring;
            p0 := v3(highest.world_position, highest_z);
            p1 := v3(next.world_position, heights[next_index]);
            add_vertex(*ring, p0);
            add_vertex(*ring, p1);

            p2_side := get_side(next_index   , opposite_index);
            p3_side := get_side(highest_index, last_index    );
            p2_cache := get_caches_for_side(p2_side);
            p3_cache := get_caches_for_side(p3_side);

            p2, p2_found := find_edge(survey, next   , opposite, heights[next_index], p2_cache);
            p3, p3_found := find_edge(survey, highest, last    , highest_z          , p3_cache);

            if p2_found then add_vertex(*ring, p2);
            if p3_found then add_vertex(*ring, p3);

            used_z_count[next_index] += 1;

            #if for_edges {
                if !p2_found then p2 = p1;
                if !p3_found then p3 = p0;
                min_z_23 := min(p2.z, p3.z);
                max_z_23 := max(p2.z, p3.z);
                diagram_colour(*wm_diagram, V4_RED);
                add_blocking_edge(survey, p2.xy, p3.xy, min_z_23, max_z_23 + survey.mover.height, survey.mover.radius);
            } else {
                if ring.edge_count < 3 continue;
                assert(ring.edge_count >= 3 && ring.edge_count <= 6);
                array_add(*survey.rings, ring);
            }
        } else if valid[next_index] && !valid[opposite_index] && valid[last_index] {
            ring : Ring;

            p0 := v3(highest.world_position, highest_z);
            p1 := v3(next.world_position, heights[next_index]);

            add_vertex(*ring, p0);
            add_vertex(*ring, p1);
            
            p2_side := get_side(next_index, opposite_index);
            p3_side := get_side(last_index, opposite_index);
            p2_cache := get_caches_for_side(p2_side);
            p3_cache := get_caches_for_side(p3_side);

            p2, p2_found := find_edge(survey, next, opposite, heights[next_index], p2_cache);
            p3, p3_found := find_edge(survey, last, opposite, heights[last_index], p3_cache);

            if p2_found then add_vertex(*ring, p2);
            if p3_found then add_vertex(*ring, p3);

            p4 := v3(last.world_position, heights[last_index]);
            add_vertex(*ring, p4);

            used_z_count[next_index] += 1;
            used_z_count[last_index] += 1;

            #if for_edges {
                edge_i0 := 1;
                if p2_found then edge_i0 += 1;
                edge_i1 := edge_i0 + 1;
                ep0 := ring.edges[edge_i0].position;
                ep1 := ring.edges[edge_i1].position;
                min_z_01 := min(ep0.z, ep1.z);
                max_z_01 := max(ep1.z, ep0.z);
                diagram_colour(*wm_diagram, V4_RED);
                add_blocking_edge(survey, ep0.xy, ep1.xy, min_z_01, max_z_01 + survey.mover.height, survey.mover.radius);
            } else {
                assert(ring.edge_count >= 3 && ring.edge_count <= 6);
                array_add(*survey.rings, ring);
            }
        } else if valid[next_index] && valid[opposite_index] && !valid[last_index] {
            ring : Ring;

            p0 := v3(highest.world_position, highest_z);
            p1 := v3(next.world_position, heights[next_index]);
            p2 := v3(opposite.world_position, heights[opposite_index]);

            add_vertex(*ring, p0);
            add_vertex(*ring, p1);
            add_vertex(*ring, p2);

            p3_side := get_side(opposite_index, last_index);
            p4_side := get_side(highest_index , last_index);
            p3_cache := get_caches_for_side(p3_side);
            p4_cache := get_caches_for_side(p4_side);
            
            p3, p3_found := find_edge(survey, opposite, last, heights[opposite_index], p3_cache);
            p4, p4_found := find_edge(survey, highest , last, highest_z              , p4_cache);

            if p3_found add_vertex(*ring, p3);
            if p4_found add_vertex(*ring, p4);

            used_z_count[next_index    ] += 1;
            used_z_count[opposite_index] += 1;

            #if for_edges {
                edge_i0 := 2;
                if p3_found then edge_i0 += 1;
                edge_i1 := (edge_i0 + 1) % ring.edge_count;
                ep0 := ring.edges[edge_i0].position;
                ep1 := ring.edges[edge_i1].position;
                min_z_01 := min(ep0.z, ep1.z);
                max_z_01 := max(ep1.z, ep0.z);
                diagram_colour(*wm_diagram, V4_RED);
                add_blocking_edge(survey, ep0.xy, ep1.xy, min_z_01, max_z_01 + survey.mover.height, survey.mover.radius);
            } else {
                assert(ring.edge_count >= 3 && ring.edge_count <= 6);
                array_add(*survey.rings, ring);
            }
        } else if !valid[next_index] && valid[opposite_index] && valid[last_index] {
            ring : Ring;

            p0 := v3(highest.world_position, highest_z);
            add_vertex(*ring, p0);

            p1_side := get_side(highest_index , next_index);
            p2_side := get_side(opposite_index, next_index);
            p1_cache := get_caches_for_side(p1_side);
            p2_cache := get_caches_for_side(p2_side);

            p1, p1_found := find_edge(survey, highest , next, highest_z              , p1_cache);
            p2, p2_found := find_edge(survey, opposite, next, heights[opposite_index], p2_cache);
            if p1_found add_vertex(*ring, p1);
            if p2_found add_vertex(*ring, p2);

            p3 := v3(opposite.world_position, heights[opposite_index]);
            p4 := v3(last.world_position, heights[last_index]);

            add_vertex(*ring, p3);
            add_vertex(*ring, p4);

            used_z_count[last_index    ] += 1;
            used_z_count[opposite_index] += 1;

            #if for_edges {
                edge_i0 := 0;
                if p1_found then edge_i0 += 1;
                edge_i1 := (edge_i0 + 1) % ring.edge_count;
                ep0 := ring.edges[edge_i0].position;
                ep1 := ring.edges[edge_i1].position;
                min_z_01 := min(ep0.z, ep1.z);
                max_z_01 := max(ep1.z, ep0.z);
                diagram_colour(*wm_diagram, V4_RED);
                add_blocking_edge(survey, ep0.xy, ep1.xy, min_z_01, max_z_01 + survey.mover.height, survey.mover.radius);
            } else {
                assert(ring.edge_count >= 3 && ring.edge_count <= 6);
                array_add(*survey.rings, ring);
            }
        } else {
            #if !for_edges {
                diagram_colour(*wm_diagram, V4_WHITE);
                p0 := v3(highest.world_position, highest_z);
                p1 := v3(next.world_position, heights[next_index]);
                p2 := v3(opposite.world_position, heights[opposite_index]);
                p3 := v3(last.world_position, heights[last_index]);

                ring : Ring;
                ring.edge_count = 4;
                ring.edges[0] = .{p0, -1};
                ring.edges[1] = .{p1, -1};
                ring.edges[2] = .{p2, -1};
                ring.edges[3] = .{p3, -1};

                array_add(*survey.rings, ring);
            }

            used_z_count[next_index    ] += 1;
            used_z_count[opposite_index] += 1;
            used_z_count[last_index    ] += 1;
        }
    }

    return V2s.{cast(s32)starting_ring_count, cast(s32)(survey.rings.count - starting_ring_count)};
}


find_edge :: (survey : *Walk_Manifold_Survey, in_pole : *Pole, out_pole : *Pole, in_z : float, cache : Cache_Pair) -> Vector3, found : bool {
    profile_function();
    if cache.read_cache {
        c := cache.read_cache;
        for c.vertices {
            if it.in_pole_z == in_z {
                c.read_count += 1;
                return it.v, it.found;
            }
        }
    }

    dir := out_pole.world_position - in_pole.world_position;

    start :=  in_pole.world_position;
    end   := out_pole.world_position;

    //return v3(start + dir*0.5, in_z);

    t := 0.5;
    step := 0.25; 

    result := v3(start, in_z);
    found := false;


    endpoint_bb := rect_from_bounds(start, end);

    contributing_walkable : [..]Walkable_Triangle;
    contributing_walkable.allocator=temp;
    for tri : survey.walkable_triangles {
        //@Speed: we could also test whether the range of possible we could
        //        find here lies inside the edges min_remove_z and max_remove_z
        if !collide(endpoint_bb, tri.bounds) continue;
        array_add(*contributing_walkable, tri);
    }

    contributing_edges : [..]Blocking_Edge;
    contributing_edges.allocator=temp;
    for edge : survey.active_blocking_edges {
        //@Speed: we could also test whether the range of possible we could
        //        find here lies inside the edges min_remove_z and max_remove_z
        bb := edge.expanded_bounds;
        if !collide(endpoint_bb, bb) continue;

        array_add(*contributing_edges, edge);
    }


    cylinder : Cylinder;
    cylinder.radius = survey.mover.radius;
    cylinder.bottom = survey.mover.leg_clearance;
    cylinder.top    = survey.mover.height;
    ca : Transform;

    rv2 := v2(survey.mover.radius);
    expanded_endpoint_bb := endpoint_bb;
    expanded_endpoint_bb.bl -= rv2; 
    expanded_endpoint_bb.wh += 2.0 * rv2;

    contributing_hulls : [..]Blocking_Hull;
    contributing_hulls.allocator=temp;
    for hull : survey.blocking_hulls {
        //@Speed: we could also test whether the range of possible we could
        //        find here lies inside the edges min_remove_z and max_remove_z
        if !collide(expanded_endpoint_bb, hull.bounds) continue;
        array_add(*contributing_hulls, hull);
    }

    contributing_blocking_tris : [..]Blocking_Triangle;
    contributing_blocking_tris.allocator = temp;
    for tri : survey.blocking_tris {
        //@Speed: we could also test whether the range of possible we could
        //        find here lies inside the edges min_remove_z and max_remove_z
        if !collide(expanded_endpoint_bb, tri.bounds) continue;
        array_add(*contributing_blocking_tris, tri);
    }

    for 0..8-1 {
        p := lerp(start, end, t);

        z_ranges : [..]Z_Range;
        z_ranges.allocator = temp;

        for tri : contributing_walkable {
            hits, height := evaluate_triangle(tri, p);
            if hits {
                insert_z_range(*z_ranges, height, survey.mover.height);
            }
        }

        for z_ranges {
            if abs(it.max - in_z) > survey.mover.max_cell_height_change * t then remove it;
        }

        if z_ranges.count > 0 {
            for edge : contributing_edges {
                if horizontal_distance_to_edge_squared(edge, p) < edge.radius_squared {
                    for z_ranges {
                        if it.max > edge.min_remove_z && it.max < edge.max_remove_z remove it;
                    }
                }
            }
        }

        ca.position.xy = p;
        for hull : contributing_hulls {
            if !inside(hull.bounds, p) continue;
            for z_ranges {
                if it.max > hull.max_remove_z || it.max < hull.min_remove_z continue;
                ca.position.z = it.max;
                if gjk(hull.points, cylinder, ca) 
                    remove it;
            }
        }

        for tri : contributing_blocking_tris {
            for z_ranges {
                if it.max > tri.max_remove_z || it.max < tri.min_remove_z then continue;
                ca.position.z = it.max;
                if gjk(.[tri.ps], cylinder, ca) remove it;
            }
        }

        //NOTE: Z RANGE NOT GUARANTEED TO BE IN ORDER AFTER THE ABOVE, due to array removal

        if z_ranges.count > 0 {
            best_z := z_ranges[0].max;
            for z_ranges {
                if it.max > best_z best_z = it.max;
            }

            result = v3(p, best_z);
            diagram_colour(*wm_diagram, V4_GREEN);
            diagram_add_point(*wm_diagram, result);
            found = true;

            t += step;
        } else {
            diagram_colour(*wm_diagram, V4_RED);
            diagram_add_point(*wm_diagram, v3(p, in_z));
            t -= step;
        }

        step *= 0.5;
    }

    if cache.write_cache {
        c := cache.write_cache;

        if c.write_count >= survey.walkable_triangles.count {
            print("WARNING: More cache writes than walkable triangles.\n");
            return result, found;
        }

        c.vertices[c.write_count] = .{result, in_z, found};
        c.write_count += 1;
    }

    return result, found;
}


rasterize_blocking_edges :: (survey : *Walk_Manifold_Survey) {
    profile_function();
    for edge : survey.active_blocking_edges {
        //@Todo @Speed: only loop over the radius expanded bounding
        //              box of the edge, instead of all poles.
        diagram_colour(*wm_diagram, V4_RED);
        diagram_add_blocking_edge(*wm_diagram, edge);

        for y : edge.min_pole_index.y..edge.max_pole_index.y {
            for x : edge.min_pole_index.x..edge.max_pole_index.x {
                pole_index := x + y*(survey.tile_count.x+1);
                pole := *survey.poles[pole_index];

                p := pole.world_position;
                dist := horizontal_distance_to_edge_squared(edge, p);

                #if DEVELOPER {
                    if pole.z_ranges.count > 0 {
                        diagram_colour(*wm_diagram, V4_ORANGE);
                        diagram_add_point(*wm_diagram, v3(p, pole.z_ranges[0].max));
                    }
                }
                if dist < edge.radius_squared {
                    for pole.z_ranges {
                        if it.max >= edge.min_remove_z && it.max <= edge.max_remove_z {
                            diagram_colour(*wm_diagram, V4_BLACK);
                            diagram_add_point(*wm_diagram, v3(p, it.max));
                            remove it;
                        }
                    }
                }
            }
        }
    }
}

rasterize_blocking_hulls :: (survey : *Walk_Manifold_Survey) {
    profile_function();

    cylinder : Cylinder; 
    cylinder.radius = survey.mover.radius; 
    cylinder.bottom = survey.mover.leg_clearance;
    cylinder.top    = survey.mover.height;
    ca : Transform;

    for hull : survey.blocking_hulls {
        for y : hull.min_pole_index.y..hull.max_pole_index.y {
            for x : hull.min_pole_index.x..hull.max_pole_index.x {
                pole_index := x + y*(survey.tile_count.x+1);
                pole := *survey.poles[pole_index];
                p := pole.world_position;
                ca.position.xy = p;

                for z_range : pole.z_ranges {
                    //@Speed: pre-compute hull min_z/max_z and use that to reject before gjk-ing.
                    ca.position.z = z_range.max;
                    if gjk(hull.points, cylinder, ca) remove z_range;
                }
            }
        }
    }
}

rasterize_blocking_tris :: (survey : *Walk_Manifold_Survey) {
    profile_function();

    cylinder : Cylinder;
    cylinder.bottom = survey.mover.leg_clearance;
    cylinder.top    = survey.mover.height;
    cylinder.radius = survey.mover.radius;

    ct : Transform;

    for tri : survey.blocking_tris {
        for y : tri.min_pole_index.y..tri.max_pole_index.y {
            for x : tri.min_pole_index.x..tri.max_pole_index.x {
                pole_index := x + y*(survey.tile_count.x+1);
                pole := *survey.poles[pole_index];
                p := pole.world_position;

                ct.position.xy = p;
                for pole.z_ranges {
                    if it.max < tri.min_remove_z || it.max > tri.max_remove_z then continue;
                    //@Speed: Seems like there must be a faster way to do this.
                    ct.position.z = it.max;
                    if gjk(.[tri.ps], cylinder, ct) remove it;
                }
            }
        }
    }
}

find_closest_point_in_ring :: (ring : *Ring, p : Vector3) -> Vector3, distance_squared : float {
    triangle_count, tris := triangulate(ring);
    triangles : []Triangle = tris;
    triangles.count = triangle_count;

    best_p : Vector3;
    best_distance2 := FLOAT32_MAX;

    for t : triangles {
        tp := get_closest_point_in_triangle(t, p);

        d2 := length_squared(tp - p);
        if d2 < best_distance2 {
            best_distance2 = d2;
            best_p = tp;
        }
    }

    return best_p, best_distance2;
}


add_rings_to_surface :: (survey : *Walk_Manifold_Survey, surface : *Walkable_Surface, r : *Ring) {
    stack : [..]*Ring;
    stack.allocator = temp;

    array_add(*stack, r);

    while stack.count {
        current := pop(*stack);
        if current.marked continue;

        current.marked = true;
        array_add(*surface.rings, current);

        for 0..current.edge_count-1 {
            next_i := current.edges[it].to;
            if next_i < 0 continue;
            next := *survey.rings[next_i];
            if !next.marked then array_add(*stack, next);
        }
    }
}

evaluate_triangle :: (tri : Walkable_Triangle, xy : Vector2) -> hits : bool, height : float {
    profile_function();
    if !inside(tri.bounds, xy) return false, 0.0;
    //@Speed pre-compute the barycentric vectors once during triangle collation time 

    w0 := cross(tri.p1.xy - xy, tri.e12) * tri.inv_twice_horz_area;
    if w0 < 0 return false, 0.0;

    w1 := cross(xy - tri.p0.xy, tri.e02) * tri.inv_twice_horz_area;
    if w1 < 0 return false, 0.0;

    w10 := w1 + w0;
    if w10 > 1.0 return false, 0.0;
    w2 := 1.0 - w10;
    return true, tri.p0.z*w0 + tri.p1.z*w1 + tri.p2.z*w2;
}

horizontal_distance_to_edge_squared :: (edge : Blocking_Edge, p : Vector2) -> float {
    to_p := p - edge.p0;
    if edge.inv_e0_len_squared > 10000.0 return length_squared(to_p);

    d := dot(to_p, edge.e0_delta);
    t := clamp(d * edge.inv_e0_len_squared, 0.0, 1.0);

    closest_p := edge.p0 + edge.e0_delta*t;

    return length_squared(closest_p - p);
}

insert_z_range :: (z_ranges : *[..]Z_Range, z_value : float, mover_height : float) {
    new_z_range : Z_Range = ---;
    new_z_range.max = z_value;
    new_z_range.min = z_value - mover_height;

    if z_ranges.count == 0 {
        array_add(z_ranges, new_z_range);
        return;
    } 

    for *test_z_level : z_ranges.* {
        if test_z_level.max <= z_value {
            //The z_value is above this test level
            if new_z_range.min <= test_z_level.max {
                //These ranges would overlap, so just expand
                // the existing range
                test_z_level.max = z_value;
            } else {
                array_insert_at(z_ranges, new_z_range, it_index);
            }
            return;
        } else {
            if test_z_level.min < z_value {
                test_z_level.min = min(test_z_level.min, new_z_range.min);
                return;
            } 
        }
    }

    //It was not above or within any existing ranges, so it must be below them all
    array_add(z_ranges, new_z_range);
}

map_edges :: (survey : *Walk_Manifold_Survey, last_rings : V2s, new_rings : V2s) {

    for last_r_i : last_rings.x..last_rings.x+last_rings.y-1 {
        last := *survey.rings[last_r_i];
        assert(last.edge_count >= 3 && last.edge_count <= 6);
        for last_i : 0..last.edge_count-1 {
            last_edge := *last.edges[last_i];
            last_next_p := *last.edges[(last_i + 1) % last.edge_count].position; 
            if last_edge.to > 0 then continue;

            for new_r_i : new_rings.x..new_rings.x+new_rings.y-1{
                new := *survey.rings[new_r_i];
                assert(new.edge_count >= 3 && new.edge_count <= 6);
                for new_i : 0..new.edge_count-1 {
                    new_edge := *new.edges[new_i];
                    new_next_p := *new.edges[(new_i + 1) % new.edge_count].position; 
                    if (last_edge.position == new_next_p && last_next_p == new_edge.position) ||
                    (last_edge.position == new_edge.position && last_next_p == new_next_p) {
                        assert(new_edge.to < 0);
                        new_edge.to = last_r_i;
                        last_edge.to = new_r_i;
                        continue last_i;
                    }
                }
            }
        }
    }
}

add_blocking_edge :: (survey : *Walk_Manifold_Survey, p0 : Vector2, p1 : Vector2, min_z : float, max_z : float, radius : float) {
    edge : Blocking_Edge = ---;

    edge.radius_squared = radius * radius; 

    edge.p0 = p0;
    edge.p1 = p1;

    edge.e0_delta = edge.p1 - edge.p0;
    edge.e0_len_squared = dot(edge.e0_delta, edge.e0_delta);


    edge.expanded_bounds = rect_from_bounds(edge.p0, edge.p1);
    radius_v2 := v2(radius);
    edge.expanded_bounds.bl -= radius_v2;
    edge.expanded_bounds.wh += radius_v2 * 2.0;

    edge.inv_e0_len_squared = 1.0 / edge.e0_len_squared;

    edge.min_pole_index, edge.max_pole_index = get_pole_index_range(survey, p0, p1, radius);

    edge.min_remove_z = min(min_z, max_z) - survey.mover.height;
    edge.max_remove_z = max(min_z, max_z) - survey.mover.leg_clearance;

    //diagram_colour(*wm_diagram, V4_RED);
    diagram_add_blocking_edge(*wm_diagram, edge);

    //diagram_colour(*wm_diagram, V4_CYAN);
    diagram_add_point(*wm_diagram, v3(p0, min_z));
    diagram_add_point(*wm_diagram, v3(p1, min_z));

    array_add(*survey.blocking_edges, edge);
}

add_blocking_tri :: (survey : *Walk_Manifold_Survey, tri : Triangle) {
    t := array_add(*survey.blocking_tris);
    //@Cleanup: This is ridiculous, but we don't have good api to gjk for tri vs. Cylinder
    t.ps.x.v[0] = tri.p0.x;
    t.ps.x.v[1] = tri.p1.x;
    t.ps.x.v[2] = tri.p2.x;
    t.ps.y.v[0] = tri.p0.y;
    t.ps.y.v[1] = tri.p1.y;
    t.ps.y.v[2] = tri.p2.y;
    t.ps.z.v[0] = tri.p0.z;
    t.ps.z.v[1] = tri.p1.z;
    t.ps.z.v[2] = tri.p2.z;
    for 3..7 {
        t.ps.x.v[it] = t.ps.x.v[0];
        t.ps.y.v[it] = t.ps.y.v[0];
        t.ps.z.v[it] = t.ps.z.v[0];
    }

    t.min_pole_index, t.max_pole_index = get_pole_index_range(survey, tri.p0.xy, tri.p1.xy, tri.p2.xy, survey.mover.radius);

    t.min_remove_z = min(tri.p0.z, min(tri.p1.z, tri.p2.z)) - survey.mover.height;
    t.max_remove_z = max(tri.p0.z, max(tri.p1.z, tri.p2.z)) - survey.mover.leg_clearance;

    t.bounds = rect_from_bounds(tri.p0.xy, tri.p1.xy, tri.p2.xy);

    diagram_thickness(*wm_diagram, 0.01);
    diagram_colour(*wm_diagram, V4_ORANGE);
    diagram_add_triangle(*wm_diagram, tri);
}

triangulate :: (ring : *Ring) -> count : s8, [4]Triangle {
    assert(ring.edge_count >= 3 && ring.edge_count <= 6);
    count : s8 = 0;
    result : [4]Triangle;

    while count < ring.edge_count-2 {
        tri := Triangle.{ring.edges[0].position, ring.edges[count+1].position, ring.edges[count+2].position};
        result[count] = tri;
        count += 1;
    }

    return count, result;
}

get_rings :: (survey : *Walk_Manifold_Survey, index_span : V2s) -> []Ring {
    assert(survey.rings.count >= index_span.x + index_span.y);
    rings : []Ring = ---;
    rings.data = survey.rings.data + index_span.x;
    rings.count = index_span.y;
    return rings;
}

get_pole_index_range :: (survey : *Walk_Manifold_Survey, p0 : Vector2, p1 : Vector2, radius := 0.0) -> min_indices : V2s, max_indices : V2s {
    min_pole_index : V2s = ---;
    max_pole_index : V2s = ---;
    inv_cell_dim := survey.inv_cell_dim;
    min_x_i_p0 := cast(s32)(((p0.x-radius) - survey.aabb.min.x) * inv_cell_dim);
    min_y_i_p0 := cast(s32)(((p0.y-radius) - survey.aabb.min.y) * inv_cell_dim);
    min_x_i_p1 := cast(s32)(((p1.x-radius) - survey.aabb.min.x) * inv_cell_dim);
    min_y_i_p1 := cast(s32)(((p1.y-radius) - survey.aabb.min.y) * inv_cell_dim);
    min_pole_index.x = clamp(min(min_x_i_p0, min_x_i_p1), 0, survey.tile_count.x);
    min_pole_index.y = clamp(min(min_y_i_p0, min_y_i_p1), 0, survey.tile_count.y);
    max_x_i_p0 := cast(s32)(((p0.x+radius) - survey.aabb.min.x) * inv_cell_dim) + 1;
    max_y_i_p0 := cast(s32)(((p0.y+radius) - survey.aabb.min.y) * inv_cell_dim) + 1;
    max_x_i_p1 := cast(s32)(((p1.x+radius) - survey.aabb.min.x) * inv_cell_dim) + 1;
    max_y_i_p1 := cast(s32)(((p1.y+radius) - survey.aabb.min.y) * inv_cell_dim) + 1;
    max_pole_index.x = clamp(max(max_x_i_p0, max_x_i_p1), 0, survey.tile_count.x);
    max_pole_index.y = clamp(max(max_y_i_p0, max_y_i_p1), 0, survey.tile_count.y);
    return min_pole_index, max_pole_index;
}

get_pole_index_range :: (survey : *Walk_Manifold_Survey, p0 : Vector2, p1 : Vector2, p2 : Vector2, radius := 0.0) -> min_indices : V2s, max_indices : V2s {
    //@Speed: would this be faster to just calculate the bounding box FIRST, then do the calculations.
    min_pole_index : V2s = ---;
    max_pole_index : V2s = ---;
    inv_cell_dim := survey.inv_cell_dim;
    min_x_i_p0 := cast(s32)(((p0.x-radius) - survey.aabb.min.x) * inv_cell_dim);
    min_y_i_p0 := cast(s32)(((p0.y-radius) - survey.aabb.min.y) * inv_cell_dim);
    min_x_i_p1 := cast(s32)(((p1.x-radius) - survey.aabb.min.x) * inv_cell_dim);
    min_y_i_p1 := cast(s32)(((p1.y-radius) - survey.aabb.min.y) * inv_cell_dim);
    min_x_i_p2 := cast(s32)(((p2.x-radius) - survey.aabb.min.x) * inv_cell_dim);
    min_y_i_p2 := cast(s32)(((p2.y-radius) - survey.aabb.min.y) * inv_cell_dim);
    min_pole_index.x = clamp(min(min(min_x_i_p0, min_x_i_p1), min_x_i_p2), 0, survey.tile_count.x);
    min_pole_index.y = clamp(min(min(min_y_i_p0, min_y_i_p1), min_y_i_p2), 0, survey.tile_count.y);
    max_x_i_p0 := cast(s32)(((p0.x+radius) - survey.aabb.min.x) * inv_cell_dim) + 1;
    max_y_i_p0 := cast(s32)(((p0.y+radius) - survey.aabb.min.y) * inv_cell_dim) + 1;
    max_x_i_p1 := cast(s32)(((p1.x+radius) - survey.aabb.min.x) * inv_cell_dim) + 1;
    max_y_i_p1 := cast(s32)(((p1.y+radius) - survey.aabb.min.y) * inv_cell_dim) + 1;
    max_x_i_p2 := cast(s32)(((p2.x+radius) - survey.aabb.min.x) * inv_cell_dim) + 1;
    max_y_i_p2 := cast(s32)(((p2.y+radius) - survey.aabb.min.y) * inv_cell_dim) + 1;
    max_pole_index.x = clamp(max(max(max_x_i_p0, max_x_i_p1), max_x_i_p2), 0, survey.tile_count.x);
    max_pole_index.y = clamp(max(max(max_y_i_p0, max_y_i_p1), max_y_i_p2), 0, survey.tile_count.y);
    return min_pole_index, max_pole_index;
}

diagram_add_ring :: (d : *Diagram, ring : Ring) {
    for 0..ring.edge_count-1 {
        next := (it + 1) % ring.edge_count;
        diagram_add_line(d, ring.edges[it].position, ring.edges[next].position);
    }
}

diagram_add_blocking_edge :: (d : *Diagram, edge : Blocking_Edge) {
    if edge.e0_len_squared < MINIMUM_EDGE_LENGTH {
        p0 := v3(edge.p0, edge.min_remove_z);
        p1 := v3(edge.p1, edge.min_remove_z);
        diagram_add_line(d, p0, p1);
    } else {
        p0 := v3(edge.p0, edge.min_remove_z);
        p1 := v3(edge.p1, edge.min_remove_z);
        p2 := v3(edge.p1, edge.max_remove_z);
        p3 := v3(edge.p0, edge.max_remove_z);

        diagram_add_quad(d, p0, p1, p2, p3);
    }
}


MINIMUM_EDGE_LENGTH :: 0.001;
