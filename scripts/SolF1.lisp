;;; Grupo 51: 81900 - Nuno Anselmo, 82047 - Andre Mendes
;;; These functions, and any other ones needed must be implemented

;;; Utilizar estes includes para os testes na versao local
;;; comentar antes de submeter
;(load "datastructures.lisp")
;(load "auxfuncs.lisp")

;;; Utilizar estes includes para a versao a submeter
; tirar o comentario antes de submeter
(load "datastructures.fas")
(load "auxfuncs.fas")

(defun isObstaclep (pos track)
  "check if there is an obstacle at position pos of the track"
  (not (nth (second pos) (nth (first pos) (track-env track))))
)

(defun isGoalp (st)
  "check if st is a goal state"
  (not (null (member (state-pos st) (track-endpositions (state-track st)) :test #'equal)))
)

(defun nextState (st act)
  "generate the nextState after state st and action act"
  (let* (
  		; Fetch current position and velocity.
  		(pos-x (first (state-pos st)))
  		(pos-y (second (state-pos st)))
  		(vel-x (first (state-vel st)))
  		(vel-y (second (state-vel st)))
  		; Set up the new velocity.
  		(new-vel-x (+ vel-x (first act)))
  		(new-vel-y (+ vel-y (second act)))
  		; Set up the new position.
  		(new-pos-x (+ pos-x new-vel-x))
  		(new-pos-y (+ pos-y new-vel-y))
  		; Make the new state without a cost.
  		(new-state
  			(make-state
	  			:pos (list new-pos-x new-pos-y)
	  			:vel (list new-vel-x new-vel-y)
	  			:action act
	  			:track (state-track st)
	  			:other (state-other st)
  			)
  		)
  	)
  	; Set up the cost of the new state.
  	(setf (state-cost new-state) (cond
  		; If the new pos is in a obstacle, rollback the position and set the velocity to zero.
  		((isObstaclep (list new-pos-x new-pos-y) (state-track st))
  			(progn
  				(setf new-vel-x 0)
  				(setf new-vel-y 0)
  				(setf new-pos-x pos-x)
  				(setf new-pos-y pos-y)
  				20
  			)
  		)
  		; If the new pos is the goal, then the score is -100
  		((isGoalp new-state) -100)
  		; Otherwise the movement costs 1
  		(t 1))
  	)
  	; Return the new state.
  	new-state
  )
)
