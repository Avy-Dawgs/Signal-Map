'''
For data processing.
'''
import math
import numpy as np
from numpy import interp, array, argmax
from DataPoint import DataPoint, DataPointCollection
from shared_types import RssiTime, LocationTime
from typing import List, Tuple, Optional


def location_interpolate(
        lat: list[float],
        lon: list[float],
        time: list[float],
        rssi_time: RssiTime
        ) -> DataPoint: 
    '''
    Interpolate location with distance.

    args: (all lists must be size 2 monotonically increasing)
        lat: lattitude list 
        lon: longitude list
        time: times corresponding to lat and lon lists
        rssi_time: RssiTime instance
    '''
    rssi_lat = interp(rssi_time.time, time, lat)
    rssi_lon = interp(rssi_time.time, time, lon)

    return DataPoint(rssi_lat, rssi_lon, rssi_time.rssi)

def location_interpolate2(
        location_time0: LocationTime,
        location_time1: LocationTime,
        rssi_time: RssiTime
        ) -> DataPoint: 
    '''
    Interpolate location with distance.

    args: (all lists must be size 2 monotonically increasing)
        lat: lattitude list 
        lon: longitude list
        time: times corresponding to lat and lon lists
        rssi_time: RssiTime instance
    '''
    t = array([location_time0.time, location_time1.time])
    rssi_lat = interp(rssi_time.time, t, array([location_time0.lat, location_time1.lat]))
    rssi_lon = interp(rssi_time.time, t, array([location_time0.lon, location_time1.lon]))

    return DataPoint(rssi_lat, rssi_lon, rssi_time.rssi)


def compute_distance_matrix(coords1: np.ndarray, coords2: np.ndarray, use_gps: bool = False, avg_lat: Optional[float] = None) -> np.ndarray:
    """Compute pairwise distance matrix.
    
    Args:
        coords1: Array of shape (n1, 2) with coordinates
        coords2: Array of shape (n2, 2) with coordinates
        use_gps: If True, treat coordinates as GPS (lon, lat) and compute distances in meters
        avg_lat: Average latitude for GPS distance conversion
    
    Returns:
        Distance matrix of shape (n1, n2)
    """
    if use_gps:
        if avg_lat is None:
            avg_lat = np.mean(coords1[:, 1])
        
        lat_to_m = 111320.0
        lon_to_m = 111320.0 * math.cos(math.radians(avg_lat))
        if abs(lon_to_m) < 1e-10:
            lon_to_m = 1e-10 if lon_to_m >= 0 else -1e-10
        
        coords1_scaled = coords1.copy()
        coords1_scaled[:, 0] *= lon_to_m
        coords1_scaled[:, 1] *= lat_to_m
        
        coords2_scaled = coords2.copy()
        coords2_scaled[:, 0] *= lon_to_m
        coords2_scaled[:, 1] *= lat_to_m
        
        diff = coords1_scaled[:, np.newaxis, :] - coords2_scaled[np.newaxis, :, :]
        distances = np.sqrt(np.sum(diff**2, axis=2))
    else:
        diff = coords1[:, np.newaxis, :] - coords2[np.newaxis, :, :]
        distances = np.sqrt(np.sum(diff**2, axis=2))
    
    return distances


def monte_carlo_hill_climbing(data_points: List[DataPoint], R_hc: float = 20.0, min_points_per_peak: int = 1) -> List[Tuple[float, float, float]]:
    """Monte Carlo hill climbing: start from each point and climb to local max.
    
    Args:
        data_points: List of DataPoint objects with lat, lon, and rssi
        R_hc: Neighbor search radius in meters
        min_points_per_peak: Minimum number of points that must converge to a peak
    
    Returns:
        List of (lat, lon, rssi) tuples for distinct attractor peaks
    """
    if not data_points:
        return []
    
    valid_points = []
    point_coords = []
    point_rssi = []
    
    for point in data_points:
        if point.latitude is not None and point.longitude is not None:
            point_coords.append([point.longitude, point.latitude])
            point_rssi.append(point.rssi)
            valid_points.append(point)
    
    n = len(point_coords)
    if n < 2:
        if n == 1:
            return [(point_coords[0][1], point_coords[0][0], point_rssi[0])]
        return []
    
    point_coords = np.array(point_coords)
    point_rssi = np.array(point_rssi)
    
    avg_lat = np.mean(point_coords[:, 1])
    distances = compute_distance_matrix(point_coords, point_coords, use_gps=True, avg_lat=avg_lat)
    
    neighbor_lists = []
    for i in range(n):
        neighbor_mask = distances[i] <= R_hc
        neighbor_mask[i] = False
        neighbor_indices = np.where(neighbor_mask)[0]
        neighbor_lists.append(neighbor_indices)
    
    attractor_cache = {}
    attractors = []
    
    for start_idx in range(n):
        if start_idx in attractor_cache:
            attractors.append(attractor_cache[start_idx])
            continue
        
        current_idx = start_idx
        visited = set()
        max_steps = 100
        
        for step in range(max_steps):
            if current_idx in visited:
                break
            visited.add(current_idx)
            
            neighbor_indices = neighbor_lists[current_idx]
            
            if len(neighbor_indices) == 0:
                break
            
            current_rssi = point_rssi[current_idx]
            neighbor_rssis = point_rssi[neighbor_indices]
            better_mask = neighbor_rssis > current_rssi
            better_indices = neighbor_indices[better_mask]
            
            if len(better_indices) == 0:
                break
            
            better_rssis = point_rssi[better_indices]
            best_idx_in_better = np.argmax(better_rssis)
            current_idx = better_indices[best_idx_in_better]
        
        attractor_cache[start_idx] = current_idx
        attractors.append(current_idx)
    
    attractor_counts = {}
    for i, attractor_idx in enumerate(attractors):
        if attractor_idx not in attractor_counts:
            attractor_counts[attractor_idx] = []
        attractor_counts[attractor_idx].append(i)
    
    valid_attractors = {idx: points for idx, points in attractor_counts.items() 
                       if len(points) >= min_points_per_peak}
    
    if not valid_attractors:
        return []
    
    peak_candidates = []
    for attractor_idx, converging_points in valid_attractors.items():
        peak_coord = point_coords[attractor_idx]
        peak_rssi = point_rssi[attractor_idx]
        peak_candidates.append((peak_coord[1], peak_coord[0], peak_rssi))
    
    if len(peak_candidates) > 1:
        merged_peaks = []
        used = set()
        merge_radius = 10.0
        
        for i, (p1_lat, p1_lon, p1_rssi) in enumerate(peak_candidates):
            if i in used:
                continue
            
            cluster = [i]
            for j in range(i + 1, len(peak_candidates)):
                if j in used:
                    continue
                p2_lat, p2_lon, p2_rssi = peak_candidates[j]
                
                dlat = abs(p2_lat - p1_lat) * 111320.0
                dlon = abs(p2_lon - p1_lon) * 111320.0 * math.cos(math.radians((p1_lat + p2_lat) / 2))
                dist = math.sqrt(dlat**2 + dlon**2)
                
                if dist <= merge_radius:
                    cluster.append(j)
                    used.add(j)
            
            cluster_peaks = [peak_candidates[k] for k in cluster]
            best_peak = max(cluster_peaks, key=lambda x: x[2])
            merged_peaks.append(best_peak)
        
        peak_candidates = merged_peaks
    
    peak_candidates.sort(key=lambda x: x[2], reverse=True)
    if not peak_candidates:
        return []
    
    strongest_rssi = peak_candidates[0][2]
    rssi_cutoff = strongest_rssi - 40
    filtered_peaks = [p for p in peak_candidates[:5] if p[2] >= rssi_cutoff]
    
    return filtered_peaks


def filter_victims_by_support(candidate_victims: List[Tuple[float, float, float]], data_points: List[DataPoint], 
                              R_support: float = 25.0, delta_local: float = 10.0, delta_global: float = 15.0, 
                              N_min: int = 3, alpha: float = 0.4, max_victims: int = 3) -> List[Tuple[float, float, float]]:
    """Filter candidate victims based on support from raw data points.
    
    Args:
        candidate_victims: List of (lat, lon, rssi) tuples
        data_points: List of DataPoint objects
        R_support: Radius in meters to search for supporting points
        delta_local: RSSI variation allowed for local support (dB)
        delta_global: RSSI threshold relative to global max (dB)
        N_min: Minimum absolute support count
        alpha: Support threshold as fraction of best candidate's support
        max_victims: Maximum number of victims to return
    
    Returns:
        Filtered list of (lat, lon, rssi) tuples
    """
    if not candidate_victims or not data_points:
        return []
    
    raw_coords = []
    raw_rssi = []
    for point in data_points:
        if point.latitude is not None and point.longitude is not None:
            raw_coords.append([point.longitude, point.latitude])
            raw_rssi.append(point.rssi)
    
    if not raw_coords:
        return []
    
    raw_coords = np.array(raw_coords)
    raw_rssi = np.array(raw_rssi)
    
    rssi_global_max = np.max(raw_rssi)
    
    victim_support_counts = []
    victim_scores = []
    victim_sharpness = []
    
    for v_lat, v_lon, v_rssi in candidate_victims:
        v_coord = np.array([[v_lon, v_lat]])
        avg_lat = v_lat
        distances = compute_distance_matrix(v_coord, raw_coords, use_gps=True, avg_lat=avg_lat)[0]
        
        effective_delta = delta_local
        if v_rssi > -50:
            effective_delta = delta_local + 5
        elif v_rssi > -60:
            effective_delta = delta_local + 3
        
        support_mask = (distances <= R_support) & (raw_rssi >= v_rssi - effective_delta)
        support_count = np.sum(support_mask)
        supporter_indices = np.where(support_mask)[0]
        
        if len(supporter_indices) > 0:
            supporter_rssis = raw_rssi[supporter_indices]
            local_median = np.median(supporter_rssis)
            sharpness = v_rssi - local_median
        else:
            sharpness = 0.0
        
        victim_support_counts.append(support_count)
        victim_sharpness.append(sharpness)
        
        support_bonus = 2.0 * math.log(max(1, support_count))
        sharpness_bonus = 0.5 * sharpness
        score = v_rssi + support_bonus + sharpness_bonus
        victim_scores.append(score)
    
    if not victim_scores:
        return []
    
    best_idx = np.argmax(victim_scores)
    best_support = victim_support_counts[best_idx]
    support_threshold = max(N_min, alpha * best_support)
    
    max_score = max(victim_scores)
    score_margin = 7.0
    sharpness_min = 3.0
    
    filtered_victims = []
    for i, (v_lat, v_lon, v_rssi) in enumerate(candidate_victims):
        support_count = victim_support_counts[i]
        score = victim_scores[i]
        sharpness = victim_sharpness[i]
        
        rssi_check = v_rssi >= (rssi_global_max - delta_global)
        support_check = support_count >= support_threshold
        score_check = score >= (max_score - score_margin)
        sharpness_check = sharpness >= sharpness_min
        
        if not (rssi_check and support_check and score_check and sharpness_check):
            continue
        
        filtered_victims.append((v_lat, v_lon, v_rssi, victim_support_counts[i], victim_scores[i], sharpness))
    
    if not filtered_victims:
        return []
    
    filtered_victims.sort(key=lambda x: x[4], reverse=True)
    
    min_separation_real = 18.0
    score_tie_margin = 2.0
    
    accepted_victims = []
    
    for i, victim_tuple in enumerate(filtered_victims):
        v_lat, v_lon, v_rssi, v_count, v_score, v_sharpness = victim_tuple
        
        if i == 0:
            accepted_victims.append((v_lat, v_lon, v_rssi))
            continue
        
        too_close = False
        for acc_lat, acc_lon, acc_rssi in accepted_victims:
            dlat = abs(v_lat - acc_lat) * 111320.0
            dlon = abs(v_lon - acc_lon) * 111320.0 * math.cos(math.radians((v_lat + acc_lat) / 2))
            dist = math.sqrt(dlat**2 + dlon**2)
            
            top_score = filtered_victims[0][4]
            is_tie = (v_score >= top_score - score_tie_margin)
            
            if dist < min_separation_real and not is_tie:
                too_close = True
                break
        
        if not too_close:
            accepted_victims.append((v_lat, v_lon, v_rssi))
        
        if len(accepted_victims) >= max_victims:
            break
    
    return accepted_victims


def determine_final_victims(data_points: List[DataPoint], max_victims: int = 3) -> List[Tuple[float, float, float]]:
    """Determine victim locations using Monte Carlo hill climbing and support filtering.
    
    Args:
        data_points: List of DataPoint objects
        max_victims: Maximum number of victims to return
    
    Returns:
        List of (lat, lon, rssi) tuples for detected victims, empty list if none found
    """
    if not data_points:
        return []
    
    candidate_victims = monte_carlo_hill_climbing(data_points, R_hc=20.0, min_points_per_peak=1)
    
    if not candidate_victims:
        return []
    
    filtered_victims = filter_victims_by_support(
        candidate_victims, data_points,
        R_support=25.0, delta_local=10.0, delta_global=15.0,
        N_min=3, alpha=0.4, max_victims=max_victims
    )
    
    return filtered_victims
