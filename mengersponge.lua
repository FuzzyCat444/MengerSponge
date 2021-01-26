 -- Creates a table representing point or vector.
function createpoint(x, y, z)
    return {
        x = x,
        y = y,
        z = z
    }
end

-- Divides a vector by its own length to set length to 1 (unit vector).
function normalize(point)
    local len = math.sqrt(point.x ^ 2 + point.y ^ 2 + point.z ^ 2)
    return createpoint(point.x / len, point.y / len, point.z / len)
end

-- Creates a ray from the origin point (ox, oy, oz)
-- with the direction unit vector (dx, dy, dz).
function createray(ox, oy, oz, dx, dy, dz)
    return {
        ox = ox,
        oy = oy,
        oz = oz,
        dx = dx,
        dy = dy,
        dz = dz
    }
end

-- Creates a table representing an intersection between a ray and a surface.
-- point = the coordinates of the hit.
-- t = the distance from the ray origin.
-- normal = vector perpendicular to hit surface.
function createintersection(point, t, normal)
    return {
        point = createpoint(point.x, point.y, point.z),
        t = t,
        normal = createpoint(normal.x, normal.y, normal.z)
    }
end

-- Creates a table representing an infinite axis-aligned plane.
-- Example: createplane("x", 7, "+") means create the plane x = 7
-- with a normal facing in the positive x direction.
function createplane(axis, value, dir)
    return {
        axis = axis,
        value = value,
        dir = dir
    }
end

-- Creates a table representing an axis-aligned bounding box.
-- position = starting corner point.
-- dimensions = width, height, length vector.
function createaabb(position, dimensions)
    return {
        position = createpoint(position.x, position.y, position.z),
        dimensions = createpoint(dimensions.x, dimensions.y, dimensions.z)
    }
end

-- Creates a table representing a camera with rays.
-- width = number of rays wide.
-- height = number of rays high.
-- fov = field of view angle in radians.
function createcamera(width, height, fov)
    local o = {}
    o.width = width
    o.height = height
    o.vecs = {}
    o.yaw = 0
    o.pitch = 0
    o.pos = createpoint(0, 0, 0)
    local camwidth = math.tan(fov / 2)
    local camheight = camwidth * height / width
    local yr = -1
    local yinc = 2 / height
    for y = 0, height - 1 do
        local xr = -1
        local xinc = 2 / width
        for x = 0, width - 1 do
            o.vecs[x + y * width] = 
                normalize(createpoint(xr * camwidth, yr * camheight, 1))
            xr = xr + xinc
        end
        yr = yr + yinc
    end
    return o
end

-- Creates a light at a certain point with color and distance (attenuation).
function createlight(point, color, distance)
    return {
        point = createpoint(point.x, point.y, point.z),
        color = { unpack(color) },
        distance = distance
    }
end

-- Given a list of lights, an intersection, and surface color, computes the 
-- color at that intersection.
function computelight(intersection, objectcolor, lights)
    local color = { 0, 0, 0 }
    for k, v in ipairs(lights) do
        local surfacetolight = createpoint(v.point.x - intersection.point.x,
                                           v.point.y - intersection.point.y,
                                           v.point.z - intersection.point.z)
        local distance = math.sqrt(surfacetolight.x ^ 2
                                 + surfacetolight.y ^ 2
                                 + surfacetolight.z ^ 2)
        local dim = math.max(0, (v.distance - distance) / v.distance)
        surfacetolight = normalize(surfacetolight)
        local dot = surfacetolight.x * intersection.normal.x +
                    surfacetolight.y * intersection.normal.y +
                    surfacetolight.z * intersection.normal.z
        if dot < 0 then
            dot = 0
        end
        color[1] = color[1] + dim * dot * v.color[1]
        color[2] = color[2] + dim * dot * v.color[2]
        color[3] = color[3] + dim * dot * v.color[3]
    end
    return { color[1] * objectcolor[1], 
             color[2] * objectcolor[2], 
             color[3] * objectcolor[3] }
end

-- Takes rays from a camera object and transforms them into world space.
function camerarays(camera)
    function rotatepitch(vec, cos, sin)
        local newvec = createpoint(vec.x, vec.y, vec.z)
        newvec.z = cos * vec.z - sin * vec.y
        newvec.y = sin * vec.z + cos * vec.y
        return newvec
    end
    function rotateyaw(vec, cos, sin)
        local newvec = createpoint(vec.x, vec.y, vec.z)
        newvec.x = cos * vec.x - sin * vec.z
        newvec.z = sin * vec.x + cos * vec.z
        return newvec
    end
    local rays = {}
    local cospitch = math.cos(-camera.pitch)
    local sinpitch = math.sin(-camera.pitch)
    local cosyaw = math.cos(-camera.yaw)
    local sinyaw = math.sin(-camera.yaw)
    for i = 0, #camera.vecs do
        local v = camera.vecs[i]
        local dirvec = rotateyaw(rotatepitch(v, cospitch, sinpitch), 
                                 cosyaw, sinyaw)
        local theray = createray(camera.pos.x, camera.pos.y, camera.pos.z, 
                                 dirvec.x, dirvec.y, dirvec.z)
        rays[i] = theray
    end
    return rays
end

-- Finds the intersection between a ray and plane.
function rayplaneintersect(ray, plane)
    if plane.axis == "x" then
        if ray.dx == 0 then
            return nil
        end
        local t = (plane.value - ray.ox) / ray.dx
        if t < 0 then
            return nil
        end
        local y = ray.oy + t * ray.dy
        local z = ray.oz + t * ray.dz
        local normal = nil
        if plane.dir == "+" then
            normal = createpoint(1, 0, 0)
        elseif plane.dir == "-" then
            normal = createpoint(-1, 0, 0)
        end
        return createintersection(createpoint(plane.value, y, z), t, normal)
    elseif plane.axis == "y" then
        if ray.dy == 0 then
            return nil
        end
        local t = (plane.value - ray.oy) / ray.dy
        if t < 0 then
            return nil
        end
        local x = ray.ox + t * ray.dx
        local z = ray.oz + t * ray.dz
        local normal = nil
        if plane.dir == "+" then
            normal = createpoint(0, 1, 0)
        elseif plane.dir == "-" then
            normal = createpoint(0, -1, 0)
        end
        return createintersection(createpoint(x, plane.value, z), t, normal)
    elseif plane.axis == "z" then
        if ray.dz == 0 then
            return nil
        end
        local t = (plane.value - ray.oz) / ray.dz
        if t < 0 then
            return nil
        end
        local x = ray.ox + t * ray.dx
        local y = ray.oy + t * ray.dy
        local normal = nil
        if plane.dir == "+" then
            normal = createpoint(0, 0, 1)
        elseif plane.dir == "-" then
            normal = createpoint(0, 0, -1)
        end
        return createintersection(createpoint(x, y, plane.value), t, normal)
    end
end

-- Intersection between ray and aabb.
function rayaabbintersect(ray, aabb)
    local s = rayplaneintersect(ray, 
        createplane("z", aabb.position.z, "-"))
    local n = rayplaneintersect(ray, 
        createplane("z", aabb.position.z + aabb.dimensions.z, "+"))
    local e = rayplaneintersect(ray, 
        createplane("x", aabb.position.x, "-"))
    local w = rayplaneintersect(ray, 
        createplane("x", aabb.position.x + aabb.dimensions.x, "+"))
    local d = rayplaneintersect(ray, 
        createplane("y", aabb.position.y, "-"))
    local u = rayplaneintersect(ray, 
        createplane("y", aabb.position.y + aabb.dimensions.y, "+"))
    
    function withinrect(x, y, px, py, dx, dy)
        return x >= px and x < px + dx and y >= py and y < py + dy
    end
    
    if s == nil or not withinrect(s.point.x, s.point.y, 
        aabb.position.x, aabb.position.y, 
        aabb.dimensions.x, aabb.dimensions.y) then
        s = nil
    end
    if n == nil or not withinrect(n.point.x, n.point.y, 
        aabb.position.x, aabb.position.y, 
        aabb.dimensions.x, aabb.dimensions.y) then
        n = nil
    end
    if e == nil or not withinrect(e.point.y, e.point.z,
        aabb.position.y, aabb.position.z, 
        aabb.dimensions.y, aabb.dimensions.z) then
        e = nil
    end
    if w == nil or not withinrect(w.point.y, w.point.z, 
        aabb.position.y, aabb.position.z, 
        aabb.dimensions.y, aabb.dimensions.z) then
        w = nil
    end
    if d == nil or not withinrect(d.point.x, d.point.z, 
        aabb.position.x, aabb.position.z, 
        aabb.dimensions.x, aabb.dimensions.z) then
        d = nil
    end
    if u == nil or not withinrect(u.point.x, u.point.z, 
        aabb.position.x, aabb.position.z, 
        aabb.dimensions.x, aabb.dimensions.z) then
        u = nil
    end
    
    local intersections = { s, n, e, w, d, u }
    
    local min = nil
    for i = 1, 6 do
        local v = intersections[i]
        if min == nil or v ~= nil and v.t < min.t then
            min = v
        end
    end
    
    return min
end

-- Computes intersection with ray and menger sponge.
-- Menger sponge has a size of 2x2x2 and has a center at the origin.
-- n = Menger sponge recursion depth.
function raymengerspongeintersect(ray, n)
    -- Takes an aabb and subdivides into 27 smaller aabbs.
    -- 7 middle boxes are removed, represents a menger iteration.
    function subdivide(aabb)
        -- Coordinates of 20 boxes without the middle ones
        subdividedindices = { { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 }, { 0, 1, 0 }, 
                              { 2, 1, 0 }, { 0, 2, 0 }, { 1, 2, 0 }, { 2, 2, 0 }, 
                              { 0, 0, 1 }, { 2, 0, 1 }, { 0, 2, 1 }, { 2, 2, 1 }, 
                              { 0, 0, 2 }, { 1, 0, 2 }, { 2, 0, 2 }, { 0, 1, 2 }, 
                              { 2, 1, 2 }, { 0, 2, 2 }, { 1, 2, 2 }, { 2, 2, 2 } }
        -- New aabbs will be 1/3 the size of the original
        local dims = createpoint(aabb.dimensions.x / 3, 
                                 aabb.dimensions.y / 3, 
                                 aabb.dimensions.z / 3)
        local pos = aabb.position
        local ret = {}
        -- Add all of the 20 new aabbs to a table and return them
        for k, v in ipairs(subdividedindices) do
            local sub = createaabb(createpoint(pos.x + v[1] * dims.x, 
                                               pos.y + v[2] * dims.y,
                                               pos.z + v[3] * dims.z), dims)
            table.insert(ret, sub)
        end
        return ret
    end
    
    -- Does actual menger sponge ray tracing and returns intersection and aabb
    -- aabbs = subdivided aabbs to do intersection with
    -- n = menger recursion depth (0 base case)
    function mengerrecurse(ray, aabbs, n)
        -- Intersection test with all 20 aabbs
        local hits = {}
        for k, v in ipairs(aabbs) do
            local hit = rayaabbintersect(ray, v)
            if hit ~= nil then
                table.insert(hits, { v, hit })
            end
        end
        -- Only closest hit is necessary
        table.sort(hits, function(a, b) return a[2].t < b[2].t end)
        
        -- Base case, return an actual intersection point (may be nil)
        if n == 0 then
            return hits[1]
        end
        
        -- Base case not reached yet, continue subdividing and raytracing
        local hit = nil
        for k, v in ipairs(hits) do
            local subdivided = subdivide(v[1])
            local recursedhit = mengerrecurse(ray, subdivided, n - 1)
            if recursedhit ~= nil then
                hit = recursedhit
                -- Don't continue raytracing with occluded aabbs
                break
            end
        end
        return hit
    end
    
    -- First raytrace is on one aabb
    local initial = createaabb(createpoint(-1, -1, -1), createpoint(2, 2, 2))
    local hit = mengerrecurse(ray, { initial }, n)
    if hit ~= nil then
        -- Return just the intersection
        return hit[2]
    end
    return nil
end