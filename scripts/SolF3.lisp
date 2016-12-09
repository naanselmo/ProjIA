;;; Grupo 51: 81900 - Nuno Anselmo, 82047 - Andre Mendes

;;; Utilizar estes includes para os testes na versao local
;;; comentar antes de submeter
;(load "datastructures.lisp")
;(load "auxfuncs.lisp")

;;; Utilizar estes includes para a versao a submeter
; tirar o comentario antes de submeter
(load "datastructures.fas")
(load "auxfuncs.fas")

;;; TAI position
(defun make-pos (c l)
  (list c l))
(defun pos-l (pos)
  (first pos))
(defun pos-c (pos)
  (second pos))

;;; TAI acceleration
(defun make-acce (c l)
  (list c l))
(defun acce-l (pos)
  (first pos))
(defun acce-c (pos)
  (second pos))

;;; TAI velocity
(defun make-vel (c l)
  (list c l))
(defun vel-l (pos)
  (first pos))
(defun vel-c (pos)
  (second pos))


;; Solution of phase 1

(defun getTrackContent (pos track)
  (nth (pos-c pos) (nth (pos-l pos) (track-env track))))

;; Pedir 0,4
(defun isObstaclep (pos track)
  "check if the position pos is an obstacle"
  (or (< (pos-l pos) 0) (< (pos-c pos) 0)
      (>= (pos-l pos) (pos-l (track-size track)))
      (>= (pos-c pos) (pos-c (track-size track)))
      (null (getTrackContent pos track))))

;; Pedir 0,4
(defun isGoalp (st)
  "check if st is a solution of the problem"
  (let ((current-position (state-pos st))
	(track (state-track st)))
    (and (member current-position (track-endpositions track) :test #'equalp)
	 T)))

;; Pedir 1,2
(defun nextState (st act)
  "generate the nextState after state st and action act from prolem"
  (let ((new-state (make-state :action act :track (state-track st))))
    (setf (state-vel new-state)
	  (make-vel (+ (vel-l (state-vel st)) (acce-l act))
		    (+ (vel-c (state-vel st)) (acce-c act))))
    (setf (state-pos new-state)
	  (make-pos (+ (pos-l (state-pos st)) (vel-l (state-vel new-state)))
		    (+ (pos-c (state-pos st)) (vel-c (state-vel new-state)))))
    (setf (state-cost new-state)
	  (cond ((isGoalp new-state) -100)
		((isObstaclep (state-pos new-state) (state-track new-state)) 20)
		(T 1)))
    (when (= (state-cost new-state) 20)
      (setf (state-vel new-state) (make-vel 0 0))
      (setf (state-pos new-state) (make-pos (pos-l (state-pos st))
					    (pos-c (state-pos st)))))
    (values new-state)))

;; Solution of phase 2

;;; Pedir
(defun nextStates (st)
  "generate all possible next states"
  (let ((successors nil))
    (dolist (act (possible-actions) successors)
      (let ((new-state (nextState st act)))
	(if (not (member new-state successors :test #'equalp))
	    (push new-state successors))))))

;;; Solucao e uma seq ordenada de estados
(defun solution (node)
  (let ((seq-states nil))
    (loop
      (when (null node)
	(return))
      (push (node-state node) seq-states)
      (setf node (node-parent node)))
    (values seq-states)))


;;; limdepthfirstsearch
(defun limdepthfirstsearch (problem lim &key cutoff?)
  "limited depth first search
     st - initial state
     problem - problem information
     lim - depth limit"
  (labels ((limdepthfirstsearch-aux (node problem lim)
	     (if (isGoalp (node-state node))
		 (solution node)
		 (if (zerop lim)
		     :cutoff
		     (let ((cutoff? nil))
		       (dolist (new-state (nextStates (node-state node)))
			 (let* ((new-node (make-node :parent node :state new-state))
				(res (limdepthfirstsearch-aux new-node problem (1- lim))))
			   (if (eq res :cutoff)
			       (setf cutoff? :cutoff)
			       (if (not (null res))
				   (return-from limdepthfirstsearch-aux res)))))
		       (values cutoff?))))))
    (let ((res (limdepthfirstsearch-aux (make-node :parent nil :state (problem-initial-state problem))
					problem
					lim)))
      (if (eq res :cutoff)
	  (if cutoff?
	      :cutoff
	      nil)
	  res))))


;iterlimdepthfirstsearch
(defun iterlimdepthfirstsearch (problem &key (lim most-positive-fixnum))
  "limited depth first search
     st - initial state
     problem - problem information
     lim - limit of depth iterations"
  (let ((i 0))
    (loop
      (let ((res (limdepthfirstsearch problem i :cutoff? T)))
	(when (and res (not (eq res :cutoff)))
	  (return res))
	(incf i)
	(if (> i lim)
	    (return nil))))))

;; Solution of phase 3

;; Heuristic
(defun getEnvContent (pos env)
  (aref env (pos-l pos) (pos-c pos)))

(defun setEnvContent (pos env val)
  (setf (aref env (pos-l pos) (pos-c pos)) val))

(defun neighbourPositions (pos)
  (let ((neighbours ())
        (actions '((0 1) (1 0) (-1 0) (0 -1) (1 -1) (-1 1) (1 1) (-1 -1))))
    (dolist (action actions)
      (setf neighbours (append neighbours (list (make-pos (+ (pos-l pos) (pos-l action)) (+ (pos-c pos) (pos-c action)))))))
    neighbours))

(defun fill-environment (track)
  (let* ((queue ())
        (trackSize (track-size track))
        (env (make-array trackSize :initial-contents (track-env track)))
        (currentPos nil)
        (currentDistance 0))
    ; Set all goals to 0 and add them to the open list.
    (dolist (goalPos (track-endpositions track))
        (setEnvContent goalPos env 0)
        (setf queue (append queue (list goalPos))))
    ; First make all NIL's M
    (loop for row from 0 below (pos-l trackSize) do
      (loop for column from 0 below (pos-c trackSize) do
        (setf currentPos (make-pos row column))
        (if (null (getEnvContent currentPos env))
          (setEnvContent currentPos env most-positive-fixnum))))
    ; While queue is not empty find all T and NIL neighbours and set their distances accodingly
    (loop while queue do
      (setf currentPos (pop queue))
      (setf currentDistance (getEnvContent currentPos env))
      (dolist (neighbour (neighbourPositions currentPos))
        (if (eq (getEnvContent neighbour env) T)
          (progn
            (setEnvContent neighbour env (+ currentDistance 1))
            (setf queue (append queue (list neighbour)))))))
    env))

; Keeps track of the heuristic environment and the track that its representing,
; so whenever we change track, the heuristic environment gets calculated again.
; (heuristic enviroment, track)
(defparameter *heuristic-table* '(nil nil))
(defun compute-heuristic (st)
	(let ((track (state-track st)))
    (if (not (eq track (second *heuristic-table*)))
      (progn
        (setf (second *heuristic-table*) track)
        (setf (first *heuristic-table*) (fill-environment track))))
    (getEnvContent (state-pos st) (first *heuristic-table*))))

;;; A*
(defun a* (problem)
  (let ((openNodes (list (make-node :state (problem-initial-state problem) :f (funcall (problem-fn-h problem) (problem-initial-state problem)) :g (state-cost (problem-initial-state problem)) :h (funcall (problem-fn-h problem) (problem-initial-state problem))))))
    (loop while openNodes do
      (let ((expandedNode (car openNodes)))
        (if (funcall (problem-fn-isGoal problem) (node-state expandedNode))
          (let ((result (solution expandedNode)))
            (return-from a* result)))
        (setf openNodes (cdr openNodes))
        (loop for nextState in (funcall (problem-fn-nextStates problem) (node-state expandedNode)) do
          (let* (
              (g (+ (node-g expandedNode) (state-cost nextState)))
              (h (funcall (problem-fn-h problem) nextState))
              (nextNode (make-node :parent expandedNode :state nextState :f (+ g h) :g g :h h))
              (pos (position (node-f nextNode) openNodes :key #'node-f :test #'<=)))
            (case pos
              ('nil
                (if (null openNodes)
                  (setf openNodes (list nextNode))
                  (nconc openNodes (list nextNode))))
              (0 (setf openNodes (cons nextNode openNodes)))
              (otherwise
                (let ((temp (nthcdr (- pos 1) openNodes)))
                  (rplacd temp (cons nextNode (cdr temp)))))))))))
  (return-from a* nil))

; Solve the following equation for m:
; (m+1)m/2 >= d + (v+1)/2 AND (m-1)m/2 < d + (v+1)/2
; Where m is the number of turns needed to travel d if starting velocity is v
; Clever maths show that the answer can be given by (after rounding):
; m = 1/2 (-1 - 2*v + sqrt(1 + 8*d + 4*v + 4*v^2))
; (defparameter *heuristic-hashtable* (make-hash-table))
; (defun fast-number-loops (d v)
;   (let ((heur (gethash d *heuristic-hashtable*)))
;     (if heur
;       heur
;       (setf (gethash d *heuristic-hashtable*) (ceiling (* (/ 1 2) (+ -1 (* -2 v) (sqrt (+ 1 (* 8 d) (* 4 v) (* 4 v v))))))))))

; Find the stopping distance if travelling at speed v and you can slow down 1 per "turn"
; (defun espaco-travar (v)
;   (/ (* (- (abs v) 1) v) 2))
;
; Find how long it takes to travel a distance d if initial speed is 0
; (defun distance-to-turns (d)
;   (/ (- (sqrt (+ (* 8 d) 1)) 1) 2))

(defun best-search (problem)
  (let ((queuedNodes (list (make-node :state (problem-initial-state problem)))) (visited (list (state-pos (problem-initial-state problem)) (state-vel (problem-initial-state problem)))))
    (if (isGoalp (problem-initial-state problem))
      (let ((result (solution (make-node :state (problem-initial-state problem)))))
        (return-from best-search result))
      (loop while queuedNodes do
        (let ((expandedNode (pop queuedNodes)))
          (loop for nextState in (nextStates (node-state expandedNode)) do
            (if (not (member (list (state-pos nextState) (state-vel nextState)) visited :test #'equal))
              (let ((nextNode (make-node :parent expandedNode :state nextState)))
                (nconc visited (list (list (state-pos nextState) (state-vel nextState))))
                (if (isGoalp (node-state nextNode))
                  (let ((result (solution nextNode)))
                    (print (list-length result))
                    (return-from best-search result))
                  (if (null queuedNodes)
                    (setf queuedNodes (list nextNode))
                    (nconc queuedNodes (list nextNode)))))))))))
  (return-from best-search nil))
