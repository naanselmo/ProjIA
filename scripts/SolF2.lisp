;;; Grupo 51: 81900 - Nuno Anselmo, 82047 - Andre Mendes

;;; Utilizar estes includes para os testes na versao local
;;; comentar antes de submeter
(load "datastructures.lisp")
(load "auxfuncs.lisp")

;;; Utilizar estes includes para a versao a submeter
; tirar o comentario antes de submeter
;(load "datastructures.fas")
;(load "auxfuncs.fas")

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
  (let ((states ()))
    (loop for action in (possible-actions) do
      (setf states (cons (nextState st action) states)))
      states))

;;; limdepthfirstsearch
(defun limdepthfirstsearch (problem lim)
  "limited depth first search
     st - initial state
     problem - problem information
     lim - depth limit"
  (let* ((initial-state (problem-initial-state problem))
         (initial-node (make-node :state initial-state)))
    ; Start the recursion.
    (recursive_limdepthfirstsearch problem initial-node  lim)))

(defun recursive_limdepthfirstsearch (problem node lim)
  "helper function for limdepthfirstsearch"
  (let ((current-state (node-state node))
        ; Loop variables
        (child-node nil)
        (result nil)
        (cutoff_occured nil))
    (cond
      ; If the current state is goal, then return the solution we accumulated so far.
      ((funcall (problem-fn-isGoal problem) current-state) (solution node))
      ; If the limit is 0, return the 'cutoff since we just ran out of dives :(.
      ; (check with the professor if its this that we have to do)
      ((zerop lim) 'cutoff)
      ; Else lets do the DFS, LETS DIVEE ONE MORE TIME!! (One more time, I gonna celebrate, Oh yeah, all right, Don't stop the diving)
      (t
        (progn
          ; Loop through all the possible states from the current state.
          (dolist (nextState (funcall (problem-fn-nextStates problem) current-state))
            ; Make the node for this nextState, with it's state being the nextState and its parent the current node.
            (setf child-node (make-node :state nextState :parent node))
            ; Add the nextState to the solutions list.
            ; Also decrement the limit by one since we just moved one floor.
            (setf result (recursive_limdepthfirstsearch problem child-node (- lim 1)))
            ; Check the result!
            (cond
              ; If the result is equal to 'cutoff, set the cutoff_occured to true.
              ((equal result 'cutoff) (setf cutoff_occured t))
              ; If the result is not failure, then return the result.
              ((not (null result)) (return-from recursive_limdepthfirstsearch result))))
          ; This is out of the loop! If cutoff occurred while we where diving the tree return 'cutoff! else return failure (NIL)
          (if cutoff_occured 'cutoff nil))))))

(defun solution(node)
  "reconstructs the solution given the goal node"
  (let ((solution-path ()))
    (loop while (not (null node)) do
      (progn
        (push (node-state node) solution-path)
        (setf node (node-parent node))))
    solution-path))

(defun iterlimdepthfirstsearch (problem &key (lim most-positive-fixnum))
  "limited depth first search
     st - initial state
     problem - problem information
     lim - limit of depth iterations"
  (let* ((iterator 0)
         (result nil))
    (loop
      (setf iterator (+ iterator 1))
      (when (>= iterator lim) (return nil))
      (setf result (limdepthfirstsearch problem iterator))
      (when (not (eq result 'cutoff)) (return result)))))
