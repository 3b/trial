#|
 This file is a part of trial
 (c) 2017 Shirakumo http://tymoon.eu (shinmera@tymoon.eu)
 Author: Nicolas Hafner <shinmera@tymoon.eu>
|#

(in-package #:org.shirakumo.fraf.trial)
(in-readtable :qtools)

(defmethod coerce-base ((base symbol))
  (if *standalone*
      (merge-pathnames (format NIL "~(~a~)/" base) (uiop:argv0))
      (merge-pathnames "data/" (asdf:system-source-directory base))))

(defmethod coerce-base ((base pathname))
  (if *standalone*
      (merge-pathnames base (uiop:argv0))
      (merge-pathnames base (asdf:system-source-directory :trial))))

(defmethod coerce-base ((base string))
  (coerce-base (parse-namestring base)))

(defvar *pools* (make-hash-table :test 'eql))

(defun pool (name &optional errorp)
  (or (gethash name *pools*)
      (when errorp (error "No pool with name ~s." name))))

(defun (setf pool) (pool name)
  (setf (gethash name *pools*) pool))

(defun remove-pool (name)
  (remhash name *pools*))

(defun list-pools ()
  (alexandria:hash-table-values *pools*))

(defclass pool ()
  ((name :initarg :name :accessor name)
   (base :accessor base)
   (assets :initform (make-hash-table :test 'eq) :accessor assets))
  (:default-initargs
   :name (error "NAME required.")
   :base (error "BASE required.")))

(defmethod initialize-instance :after ((pool pool) &key base)
  (setf (base pool) base))

(defmethod (setf base) (base (pool pool))
  (setf (slot-value pool 'base) (coerce-base base)))

(defmacro define-pool (name &body initargs)
  `(eval-when (:compile-toplevel :load-toplevel :execute)
     (cond ((pool ',name)
            (reinitialize-instance (pool ',name) ,@initargs))
           (T
            (setf (pool ',name) (make-instance 'pool :name ',name ,@initargs))))
     ',name))

(defmethod asset ((pool pool) name &optional errorp)
  (or (gethash name (assets pool))
      (when errorp (error "No asset with name ~s on pool ~a." name pool))))

(defmethod asset ((pool symbol) name &optional errorp)
  (let ((pool (pool pool errorp)))
    (when pool (asset pool name errorp))))

(defmethod (setf asset) ((asset asset) (pool pool) name)
  (setf (gethash name (assets pool)) asset))

(defmethod (setf asset) ((asset asset) (pool symbol) name)
  (setf (asset (pool pool T) name) asset))

(defmethod list-assets ((pool pool))
  (alexandria:hash-table-values (assets pool)))

(defmethod finalize ((pool pool))
  (mapc #'finalize (list-assets pool)))

(defun substitute-asset-paths (tree pool)
  (typecase tree
    (cons (cons (substitute-asset-paths (car tree) pool)
                (substitute-asset-paths (cdr tree) pool)))
    (pathname (merge-pathnames tree (base pool)))
    (T tree)))

(defmacro define-asset (type (pool name) inputs &rest initargs)
  (let ((loaded (gensym "GENSYM"))
        (asset (gensym "ASSET")))
    (substitute-asset-paths inputs (pool pool))
    `(let ((,loaded)
           (,asset (asset ',pool ',name)))
       (cond (,asset
              (when (resource ,asset)
                (offload ,asset)
                (setf ,loaded T))
              (reinitialize-instance ,asset :inputs (list ,@inputs) ,@initargs)
              (when ,loaded
                (load ,asset)))
             (T
              (setf (asset ',pool ',name)
                    (make-asset ',type (list ,@inputs) ,@initargs))))
       ',name)))
