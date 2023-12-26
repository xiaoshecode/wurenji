import cv2
import numpy as np
import time

class circle_detector:
    
    def __init__(self):

        # thresholds for purple color detection (in HSV color space)
        self.purple_lower = np.array([120, 50, 50])
        self.purple_upper = np.array([180, 255, 255])

        # lower bound of valid patch area
        self.min_patch_area = 2000

        # patch filtering thresholds
        self.aspect_ratio_l = 0.27
        self.aspect_ratio_h = 0.6
        self.filling_rate_l = 0.38
        self.filling_rate_h = 0.55

        # patch matching thresholds
        self.shape_max_w_dev = 0.2
        self.shape_max_h_dev = 0.2
        self.match_max_x_dev = 0.2
        self.match_y_dev_h = 4.0
        self.match_y_dev_l = 2.8

        # strong matching
        self.strong_match_thresh = 0.75

        # for the cooperation of different functions
        self.thresh_img = None
        self.num_components = 0
        self.stats = None

        # for distance estimation
        self.radius_inner = 140.0 / 2.0
        self.radius_outer = 155.0 / 2.0
        
    def _thresh(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Create a mask for non-purple pixels
        mask = cv2.inRange(hsv_img, self.purple_lower, self.purple_upper)

        # Apply the mask to zero out non-purple pixels
        img = cv2.bitwise_and(img, img, mask=mask)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.thresh_img = cv2.threshold(img, 3, 255, cv2.THRESH_BINARY)[1]

    def _patch(self):
        self.num_components, _, self.stats, _ = cv2.connectedComponentsWithStats(self.thresh_img)
        # filter small patches
        large_components = self.stats[self.stats[:, cv2.CC_STAT_AREA] > self.min_patch_area]
        # sort and filter the background
        self.stats = np.array(sorted(large_components, key=lambda x:x[cv2.CC_STAT_WIDTH]*x[cv2.CC_STAT_HEIGHT]))[:-1]
        # print("self.stats=",self.stats)
        
        self.num_components = len(self.stats)

    def _extract_patch(self, img, index):
        y1 = self.stats[index, cv2.CC_STAT_LEFT]
        y2 = y1 + self.stats[index, cv2.CC_STAT_WIDTH]
        x1 = self.stats[index, cv2.CC_STAT_TOP]
        x2 = x1 + self.stats[index, cv2.CC_STAT_HEIGHT]
        ret = img[x1:x2, y1:y2]
        return ret

    def _filter_shape(self, patch, index):
        # filter 1: the aspect ratio
        h, w = patch.shape[0], patch.shape[1]
        ratio = float(h) / float(w)
        if ratio > self.aspect_ratio_h or ratio < self.aspect_ratio_l:

            return False

        # filter 2: the filling rate
        h = self.stats[index, cv2.CC_STAT_HEIGHT]
        w = self.stats[index, cv2.CC_STAT_WIDTH]
        patch_area = self.stats[index, cv2.CC_STAT_AREA]
        filling_rate = float(patch_area) / float(h * w)
        if filling_rate > self.filling_rate_h or filling_rate < self.filling_rate_l:
            return False

        return True

    def _match_patches(self):
        # exit()
        match_matrix = np.zeros(shape=(self.num_components, self.num_components))
        match_table = [[] for _ in range(self.num_components)]
        for x in range(self.num_components):
            for y in range(x + 1, self.num_components):
                patch1, patch2 = self.stats[x,:], self.stats[y,:]
                w1, w2 = patch1[cv2.CC_STAT_WIDTH], patch2[cv2.CC_STAT_WIDTH]
                h1, h2 = patch1[cv2.CC_STAT_HEIGHT], patch2[cv2.CC_STAT_HEIGHT]
                w_dev, h_dev = float(abs(w1 - w2)) / float(w1), float(abs(h1 - h2)) / float(h1)
                match_shape = w_dev < self.shape_max_w_dev and h_dev < self.shape_max_h_dev
                if not match_shape:
                    continue
                    
                # x direction
                x1, x2 = patch1[cv2.CC_STAT_LEFT], patch2[cv2.CC_STAT_LEFT]
                if float(abs(x1 - x2)) / float(w1) > self.match_max_x_dev:
                    continue

                t1, t2 = patch1[cv2.CC_STAT_TOP], patch2[cv2.CC_STAT_TOP]
                temp = float(abs(t1 - t2)) / float(h1)
                if temp > self.match_y_dev_h or temp < self.match_y_dev_l:
                    continue
                
                match_matrix[x, y] = 1
                match_matrix[y, x] = 1
                match_table[x].append(y)
                match_table[y].append(x)

        # check match matrix
        check_failed = False
        reduced_mm = np.sum(match_matrix, axis=1)
        for i in range(self.num_components):
            if reduced_mm[i] != 1:
                check_failed = True
        
        if check_failed:
            # means: some patches have matched more than one, or some have matched none

            # first, deal with the multi-match cases with stronger but more resource consuming algorithms
            for i, partners in enumerate(match_table):
                if len(partners) <= 1:
                    continue
                # the case where more than one matches exist
                max_score = 0
                max_match = 0
                for match in partners:
                    # perform strong match between the weak-matched pattern and the original pattern
                    score = self._strong_match(match, i)
                    if score > max_score:
                        max_score = score
                        max_match = match

                # remove the unwanted matches
                for match in partners:
                    if match != max_match:
                        match_table[match].remove(max_match)
                        match_table[max_match].remove(match)
                        match_matrix[match, max_match] = 0
                        match_matrix[max_match, match] = 0

            # then, remove the unmatched ones
            reduced_mm = np.sum(match_matrix, axis=1)

        new_match_table = np.array([match_table[i][0] if len(match_table[i]) > 0 else -1 for i in range(self.num_components)])

        circles = []
        visited = np.zeros_like(reduced_mm)
        for i in range(self.num_components):
            if visited[i]: 
                continue
            if new_match_table[i] < 0:
                continue
            visited[match_table[i]] = 1
            patch1, patch2 = self.stats[i], self.stats[new_match_table[i]]
            t1, t2 = patch1[cv2.CC_STAT_TOP], patch2[cv2.CC_STAT_TOP]
            w1, w2 = patch1[cv2.CC_STAT_WIDTH], patch2[cv2.CC_STAT_WIDTH]
            x1, x2 = patch1[cv2.CC_STAT_LEFT], patch2[cv2.CC_STAT_LEFT]
            h1, h2 = patch1[cv2.CC_STAT_HEIGHT], patch2[cv2.CC_STAT_HEIGHT]
            if t1 < t2:
                # higher, lower = patch1, patch2
                top = t1
                bottom = t2 + patch2[cv2.CC_STAT_HEIGHT]
            else:
                # higher, lower = patch2, patch1
                top = t2
                bottom = t1 + patch1[cv2.CC_STAT_HEIGHT]
            diameter = bottom - top
            radius = int(diameter / 2)
            center_y = int((top + bottom) / 2)
            center_x = int((x1 + x2 + w1/2 + w2/2)/2)

            circles.append(np.array([radius, center_x, center_y]))

        return circles

    def _strong_match(self, ind1, ind2):
        patch1, patch2 = self.stats[ind1], self.stats[ind2]
        t1, t2 = patch1[cv2.CC_STAT_TOP], patch2[cv2.CC_STAT_TOP]
        w1, w2 = patch1[cv2.CC_STAT_WIDTH], patch2[cv2.CC_STAT_WIDTH]
        x1, x2 = patch1[cv2.CC_STAT_LEFT], patch2[cv2.CC_STAT_LEFT]
        h1, h2 = patch1[cv2.CC_STAT_HEIGHT], patch2[cv2.CC_STAT_HEIGHT]
        shape1 = (w1, h1)
        patch1 = self.thresh_img[t1:t1+h1, x1:x1+w1]
        patch2 = self.thresh_img[t2:t2+h2, x2:x2+w2]
        patch2 = cv2.resize(patch2, shape1, interpolation=cv2.INTER_NEAREST)[::-1, ::-1]
        score = np.sum(patch1 == patch2) / (h1 * w1)
        # cv2.imshow('patches', np.concatenate([patch1, patch2], axis=0))
        # cv2.waitKey(0)
        return score

    def _filter_all_patches(self):
        filtered_stats = []
        for i in range(self.num_components):
            patch = self._extract_patch(self.thresh_img, i)
            # print("patch=",patch)
            # print(patch.sum())
            # # exit()
            # print(self.stats)
            # exit()
            if self._filter_shape(patch, i):

                filtered_stats.append(self.stats[i])
        # print(filtered_stats)
        # exit()
        filtered_stats = np.array(filtered_stats)
        self.stats = filtered_stats
        self.num_components = len(self.stats)

    def detect(self, img):
        """Detect All unobstructed circles in the image

        Args:
            img: Input image to be detected, in the format of a numpy array (or opencv image)

        Returns:
            A list of circles, each represented with a numpy array with three elements:
            [radius, center_x, center_y], in the unit of pixels
        """
        # generate patches
        self._thresh(img)
        self._patch()

        # filter patches
        self._filter_all_patches()
        self.circles = self._match_patches()
        return self.circles

    def label_circles(self, img):
        for circle in self.circles:
            img = cv2.circle(img, (circle[1], circle[2]), circle[0], (0, 255, 0), 2)
        return img

    def label_patches(self, img):
        img = self.label_circles(img)
        for i in range(self.num_components):
            img = cv2.rectangle(
                img, 
                (self.stats[i, cv2.CC_STAT_LEFT], self.stats[i, cv2.CC_STAT_TOP]),
                (self.stats[i, cv2.CC_STAT_LEFT] + self.stats[i, cv2.CC_STAT_WIDTH], self.stats[i, cv2.CC_STAT_TOP] + self.stats[i, cv2.CC_STAT_HEIGHT]),
                (255, 0, 0), 2,
            )
            txt_x = self.stats[i, cv2.CC_STAT_LEFT] + self.stats[i, cv2.CC_STAT_WIDTH] // 2
            txt_y = self.stats[i, cv2.CC_STAT_TOP] + self.stats[i, cv2.CC_STAT_HEIGHT] // 2
            #cv2.putText(img, f"obj {i}", (txt_x, txt_y), fontScale=3, fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(255, 0, 0), thickness=2)
        return img
            

if __name__ == "__main__":
    import argparse
    args = argparse.ArgumentParser()
    args.add_argument("--img", type=str)
    args = args.parse_args()

    while True:
        d = circle_detector()
        img = cv2.imread("/root/catkin_ws/src/Decodertest/pictures/111.jpg")
        circles = d.detect(img)
        img = d.label_patches(img)
        cv2.imwrite("labeled.png", img)
        print("circles=",circles)
        # cv2.imshow("Labeled Image", img)
        time.sleep(5)
        
        
    cv2.waitKey(0)
