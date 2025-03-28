#include "BYTETracker.h"
#include "ros/ros.h"
#include <fstream>

BYTETracker::BYTETracker(float track_th, float high_th, float match_th, int frame_rate, int time_lost)
{
	track_thresh = track_th;
	high_thresh = high_th;
	match_thresh = match_th;

	frame_id = 0;
	max_time_lost = 100;
}

BYTETracker::~BYTETracker()
{
}

vector<STrack> BYTETracker::update(const vector<Object>& objects, bool reset_flag, int target_id, int imgWidth, int imgHeight, int resetCenter, float y_th)
{
	this->frame_id++;
	vector<STrack> activated_stracks; // 正在运行的tracker,对应 strack_pool 中的 tracked_stracks
	vector<STrack> refind_stracks;    // 对应 strack_pool 中的 self.lost_stracks，重新激活 track 后找到的tracker
	vector<STrack> removed_stracks;
	vector<STrack> lost_stracks;      // strack_pool中没有匹配到的track
	vector<STrack> detections;        // d_high
	vector<STrack> detections_low;    // d_low
	// vector<STrack> detections_out;    // 出画部分
	vector<STrack> detections_cp;     // d_high的copy
	vector<STrack> tracked_stracks_swap;
	vector<STrack> resa, resb;
	vector<STrack> output_stracks;
	vector<STrack*> unconfirmed;      // if is_activated=false , in first time is_activated=false
	vector<STrack*> tracked_stracks;  // track list
	vector<STrack*> strack_pool;      // merge tracked track and lost track
	vector<STrack*> r_tracked_stracks;// strack_pool中没有被匹配到的track

	////// Step 1: Get detections //////
	for (int i = 0; i < objects.size(); i++)
	{
		if (objects[i].label != 0) continue;
		if (objects[i].rect.height / imgHeight > y_th) continue;
		vector<float> tlbr_;
		tlbr_.resize(4);
		tlbr_[0] = objects[i].rect.x;
		tlbr_[1] = objects[i].rect.y;
		tlbr_[2] = objects[i].rect.x + objects[i].rect.width;
		tlbr_[3] = objects[i].rect.y + objects[i].rect.height;
		float score = objects[i].prob;
		STrack strack(STrack::tlbr_to_tlwh(tlbr_), score);
		// if ((objects[i].rect.x < 50) || (objects[i].rect.x + 200 > imgWidth))
		// {
		// 	detections_out.push_back(strack);				
		// }
		if (score >= track_thresh)
		{
			detections.push_back(strack);
		}
		else
		{
			detections_low.push_back(strack);
		}
	}

	for (int i = 0; i < this->tracked_stracks.size(); i++)
	{
		if (!this->tracked_stracks[i].is_activated)
			unconfirmed.push_back(&this->tracked_stracks[i]);
		else
			tracked_stracks.push_back(&this->tracked_stracks[i]);
	}

	////// Step 2: First association //////
	strack_pool = joint_stracks(tracked_stracks, this->lost_stracks);
	STrack::multi_predict(strack_pool, this->kalman_filter);
	vector<vector<float> > dists;
	int dist_size = 0, dist_size_size = 0;
	dists = iou_distance(strack_pool, detections, dist_size, dist_size_size);

	vector<vector<int>> matches;
	vector<int> u_track, u_detection;
	linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);

	// for (int i = 0; i < detections_out.size();i++ ){
	// 	STrack *det = &detections_out[i];
	// 	det->update(*det, this->frame_id);
	// }

	for (int i = 0; i < matches.size(); i++)
	{
		STrack *track = strack_pool[matches[i][0]];
		STrack *det = &detections[matches[i][1]];
		if (track->state == TrackState::Tracked)
		{
			track->update(*det, this->frame_id);
			activated_stracks.push_back(*track);
		}
		else
		{
			track->re_activate(*det, this->frame_id);
			refind_stracks.push_back(*track);
		}
	}

	////////////////// Step 3: Second association, using low score dets //////////////////
	for (int i = 0; i < u_detection.size(); i++)
	{
		detections_cp.push_back(detections[u_detection[i]]);
	}
	detections.clear();
	detections.assign(detections_low.begin(), detections_low.end());//将detections_low的元素赋值到当前的detections容器中
	
	for (int i = 0; i < u_track.size(); i++)
	{
		if (strack_pool[u_track[i]]->state == TrackState::Tracked)
		{
			r_tracked_stracks.push_back(strack_pool[u_track[i]]);//找出strack_pool中没有被匹配到的track（这帧目标被遮挡的情况）
		}
	}

	dists.clear();
	dists = iou_distance(r_tracked_stracks, detections, dist_size, dist_size_size);

	matches.clear();
	u_track.clear();
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.5, matches, u_track, u_detection);

	//在低置信度的检测框中再次与没有被匹配到的 track 做 IOU 匹配
	for (int i = 0; i < matches.size(); i++)
	{
		STrack *track = r_tracked_stracks[matches[i][0]];
		STrack *det = &detections[matches[i][1]];
		if (track->state == TrackState::Tracked)
		{
			track->update(*det, this->frame_id);
			activated_stracks.push_back(*track);
		}
		else{////////////////// Step 1: Get detections //////////////////
			this->frame_id++;
			vector<STrack> activated_stracks;
			vector<STrack> refind_stracks;
			vector<STrack> removed_stracks;
			vector<STrack> lost_stracks;
			vector<STrack> detections;
			vector<STrack> detections_low;

			vector<STrack> detections_cp;
			vector<STrack> tracked_stracks_swap;
			vector<STrack> resa, resb;
			vector<STrack> output_stracks;

			vector<STrack*> unconfirmed;
			vector<STrack*> tracked_stracks;
			vector<STrack*> strack_pool;
			vector<STrack*> r_tracked_stracks;

			for (int i = 0; i < objects.size(); i++)
			{
				if (objects[i].label != 0) continue;
				if (objects[i].rect.height / imgHeight > y_th) continue;
				vector<float> tlbr_;
				tlbr_.resize(4);
				tlbr_[0] = objects[i].rect.x;
				tlbr_[1] = objects[i].rect.y;
				tlbr_[2] = objects[i].rect.x + objects[i].rect.width;
				tlbr_[3] = objects[i].rect.y + objects[i].rect.height;
				float score = objects[i].prob;
				STrack strack(STrack::tlbr_to_tlwh(tlbr_), score);
				// if ((objects[i].rect.x < 50) || (objects[i].rect.x + 200 > imgWidth))
				// {
				// 	detections_out.push_back(strack);				
				// }
				if (score >= track_thresh)
				{
					detections.push_back(strack);
				}
				else
				{
					detections_low.push_back(strack);
				}
			}

			// Add newly detected tracklets to tracked_stracks
			for (int i = 0; i < this->tracked_stracks.size(); i++)
			{
				if (!this->tracked_stracks[i].is_activated)
					unconfirmed.push_back(&this->tracked_stracks[i]);
				else
					tracked_stracks.push_back(&this->tracked_stracks[i]);
			}

			////////////////// Step 2: First association, with IoU //////////////////
			strack_pool = joint_stracks(tracked_stracks, this->lost_stracks);
			STrack::multi_predict(strack_pool, this->kalman_filter);

			vector<vector<float> > dists;
			int dist_size = 0, dist_size_size = 0;
			dists = iou_distance(strack_pool, detections, dist_size, dist_size_size);

			vector<vector<int> > matches;
			vector<int> u_track, u_detection;
			linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);

			for (int i = 0; i < matches.size(); i++)
			{
				STrack *track = strack_pool[matches[i][0]];
				STrack *det = &detections[matches[i][1]];
				if (track->state == TrackState::Tracked)
				{
					track->update(*det, this->frame_id);
					activated_stracks.push_back(*track);
				}
				else
				{
					track->re_activate(*det, this->frame_id);
					refind_stracks.push_back(*track);
				}
			}

			////////////////// Step 3: Second association, using low score dets //////////////////
			for (int i = 0; i < u_detection.size(); i++)
			{
				detections_cp.push_back(detections[u_detection[i]]);
			}
			detections.clear();
			detections.assign(detections_low.begin(), detections_low.end());
			
			for (int i = 0; i < u_track.size(); i++)
			{
				if (strack_pool[u_track[i]]->state == TrackState::Tracked)
				{
					r_tracked_stracks.push_back(strack_pool[u_track[i]]);
				}
			}

			dists.clear();
			dists = iou_distance(r_tracked_stracks, detections, dist_size, dist_size_size);

			matches.clear();
			u_track.clear();
			u_detection.clear();
			linear_assignment(dists, dist_size, dist_size_size, 0.5, matches, u_track, u_detection);

			for (int i = 0; i < matches.size(); i++)
			{
				STrack *track = r_tracked_stracks[matches[i][0]];
				STrack *det = &detections[matches[i][1]];
				if (track->state == TrackState::Tracked)
				{
					track->update(*det, this->frame_id);
					activated_stracks.push_back(*track);
				}
				else
				{
					track->re_activate(*det, this->frame_id);
					refind_stracks.push_back(*track);
				}
			}

			for (int i = 0; i < u_track.size(); i++)
			{
				STrack *track = r_tracked_stracks[u_track[i]];
				if (track->state != TrackState::Lost)
				{
					track->mark_lost();
					lost_stracks.push_back(*track);
				}
			}

			// Deal with unconfirmed tracks, usually tracks with only one beginning frame
			detections.clear();
			track->re_activate(*det, this->frame_id);
			refind_stracks.push_back(*track);
		}
	}
	// 如果 track 经过两次匹配之后还没有匹配到 box 的话，就标记为丢失了
	for (int i = 0; i < u_track.size(); i++)
	{
		STrack *track = r_tracked_stracks[u_track[i]];
		if (track->state != TrackState::Lost)
		{
			track->mark_lost();
			lost_stracks.push_back(*track);
		}
	}

	// Deal with unconfirmed tracks, usually tracks with only one beginning frame
	// 处理第一次匹配时没有被track匹配的检测框（一般是这个检测框第一次出现的情形）
	detections.clear();
	detections.assign(detections_cp.begin(), detections_cp.end());

	dists.clear();
	dists = iou_distance(unconfirmed, detections, dist_size, dist_size_size);

	matches.clear();
	vector<int> u_unconfirmed;
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.7, matches, u_unconfirmed, u_detection);

	for (int i = 0; i < matches.size(); i++)
	{
		unconfirmed[matches[i][0]]->update(detections[matches[i][1]], this->frame_id);
		activated_stracks.push_back(*unconfirmed[matches[i][0]]);
	}
	//匹配不上的unconfirmed_track直接删除
	for (int i = 0; i < u_unconfirmed.size(); i++)
	{
		STrack *track = unconfirmed[u_unconfirmed[i]];
		track->mark_removed();
		removed_stracks.push_back(*track);
	}

	////////////////// Step 4: Init new stracks //////////////////
	// 经过上面这些步骤后，如果还有没有被匹配的检测框，说明可能画面中新来了一个物体，
    // 那么就直接将他视为一个新的track，但是这个track的状态并不是激活态。
    // 在下一次循环的时候会先将它放到unconfirmed_track中去，
    // 然后根据有没有框匹配它来决定是激活还是丢掉
	for (int i = 0; i < u_detection.size(); i++)
	{
		STrack *track = &detections[u_detection[i]];
		if (track->score < this->high_thresh) continue;
		track->activate(this->kalman_filter, this->frame_id);
		activated_stracks.push_back(*track);
	}

	////////////////// Step 5: Update state //////////////////
	// 对于丢失目标的track来说，判断它丢失的帧数是不是超过了buffer缓冲帧数，超过就删除
	for (int i = 0; i < this->lost_stracks.size(); i++)
	{
		if (this->frame_id - this->lost_stracks[i].end_frame() > this->max_time_lost)
		{
			this->lost_stracks[i].mark_removed();
			removed_stracks.push_back(this->lost_stracks[i]);
		}
	}
	//指上一帧匹配上的track
	for (int i = 0; i < this->tracked_stracks.size(); i++)
	{
		if (this->tracked_stracks[i].state == TrackState::Tracked)
		{
			tracked_stracks_swap.push_back(this->tracked_stracks[i]);
		}
	}
	this->tracked_stracks.clear();
	this->tracked_stracks.assign(tracked_stracks_swap.begin(), tracked_stracks_swap.end());
	// 加上这一帧新激活的track（两次匹配到的track，以及由unconfirm状态变为激活态的track
	this->tracked_stracks = joint_stracks(this->tracked_stracks, activated_stracks);
	// 加上丢帧目标重新被匹配的track
	this->tracked_stracks = joint_stracks(this->tracked_stracks, refind_stracks);
	// 在经过这一帧的匹配之后如果被重新激活的话就将其移出列表
	this->lost_stracks = sub_stracks(this->lost_stracks, this->tracked_stracks);
	// 将这一帧丢掉的track添加进列表
	for (int i = 0; i < lost_stracks.size(); i++)
	{
		this->lost_stracks.push_back(lost_stracks[i]);
	}
	// 如果在缓冲帧数内一直没有被匹配上被 remove 的话也将其移出 lost_stracks 列表
	this->lost_stracks = sub_stracks(this->lost_stracks, this->removed_stracks);
	// 更新被移除的track列表
	for (int i = 0; i < removed_stracks.size(); i++)
	{
		this->removed_stracks.push_back(removed_stracks[i]);
	}
	
	remove_duplicate_stracks(resa, resb, this->tracked_stracks, this->lost_stracks); // 移除两段 stracks 重合度高的部分
	this->tracked_stracks.clear();
	this->tracked_stracks.assign(resa.begin(), resa.end());
	this->lost_stracks.clear();
	this->lost_stracks.assign(resb.begin(), resb.end());
	// 得到最终的结果，也就是成功追踪的track序列
	int count = 0;
	for (int i = 0; i < this->tracked_stracks.size(); i++)
	{
		if (this->tracked_stracks[i].is_activated)
		{
			if(reset_flag) 
			{
				if (this->tracked_stracks[i].tlwh[0] > imgWidth / 2 - resetCenter 
					&& this->tracked_stracks[i].tlwh[0] + this->tracked_stracks[i].tlwh[2] < imgWidth / 2 + resetCenter)
				{
					this->tracked_stracks[i].track_id = target_id;
				}
				else
				{
					count++;
					this->tracked_stracks[i].track_id = target_id + count;
				}
			}
			// if(this->tracked_stracks.size() == 1){
			// 	this->tracked_stracks[i].track_id = target_id;
			// }
			output_stracks.push_back(this->tracked_stracks[i]);
		}
	}
	return output_stracks;
}
